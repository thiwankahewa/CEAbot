#!/usr/bin/env python3
"""Solve and validate eye-in-hand calibration observations."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from bench_robot.handeye_capture import DICTIONARIES, rotation_matrix_to_quaternion


METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def pose_dict_to_matrix(pose):
    quaternion = pose["quaternion_xyzw"]
    x, y, z, w = np.asarray(
        [quaternion[name] for name in "xyzw"], dtype=np.float64
    )
    x, y, z, w = np.array([x, y, z, w]) / np.linalg.norm([x, y, z, w])
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    result[:3, 3] = [pose["translation"][name] for name in "xyz"]
    return result


def mean_rotation(rotations):
    total = np.sum(rotations, axis=0)
    u, _, vt = np.linalg.svd(total)
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def transform_result(transform):
    quaternion = rotation_matrix_to_quaternion(transform[:3, :3])
    rotation_vector, _ = cv2.Rodrigues(transform[:3, :3])
    return {
        "translation_xyz_m": [float(value) for value in transform[:3, 3]],
        "quaternion_xyzw": [float(value) for value in quaternion],
        "rotation_vector_rad": [float(value) for value in rotation_vector.ravel()],
        "matrix": [[float(value) for value in row] for row in transform],
    }


def redetect_board_poses(session_dir, document, maximum_tf_age):
    board_data = document["board"]
    camera_data = document["camera_info"]
    dictionary = cv2.aruco.getPredefinedDictionary(
        DICTIONARIES[board_data["dictionary"]]
    )
    board = cv2.aruco.CharucoBoard(
        (board_data["squares_x"], board_data["squares_y"]),
        board_data["square_length_m"],
        board_data["marker_length_m"],
        dictionary,
    )
    detector = cv2.aruco.CharucoDetector(board)
    camera_matrix = np.asarray(camera_data["k"]).reshape(3, 3)
    distortion = np.asarray(camera_data["d"])
    robot_poses, board_poses, observation_indices, reprojection_errors = [], [], [], []

    for observation in document["observations"]:
        if abs(float(observation.get("robot_tf_age_seconds", 0.0))) > maximum_tf_age:
            continue
        image = cv2.imread(str(session_dir / observation["image"]))
        corners, ids, _, _ = detector.detectBoard(image)
        if ids is None or len(ids) < 8:
            continue
        object_points = board.getChessboardCorners()[ids.flatten()]
        success, rotation_vector, translation_vector = cv2.solvePnP(
            object_points,
            corners.reshape(-1, 2),
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            continue
        rotation, _ = cv2.Rodrigues(rotation_vector)
        camera_from_board = np.eye(4)
        camera_from_board[:3, :3] = rotation
        camera_from_board[:3, 3] = translation_vector.ravel()
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            translation_vector,
            camera_matrix,
            distortion,
        )
        rmse = float(
            np.sqrt(
                np.mean(
                    np.square(projected.reshape(-1, 2) - corners.reshape(-1, 2))
                )
            )
        )
        robot_poses.append(pose_dict_to_matrix(observation["base_to_end_effector"]))
        board_poses.append(camera_from_board)
        observation_indices.append(int(observation["index"]))
        reprojection_errors.append(rmse)
    return robot_poses, board_poses, observation_indices, reprojection_errors


def score_solution(robot_poses, board_poses, end_effector_from_camera):
    base_from_boards = np.asarray(
        [
            robot_pose @ end_effector_from_camera @ board_pose
            for robot_pose, board_pose in zip(robot_poses, board_poses)
        ]
    )
    translations = base_from_boards[:, :3, 3]
    center = np.median(translations, axis=0)
    translation_errors_mm = np.linalg.norm(translations - center, axis=1) * 1000
    rotation_center = mean_rotation(base_from_boards[:, :3, :3])
    rotation_errors_deg = np.degrees(
        np.arccos(
            np.clip(
                [
                    (np.trace(rotation_center.T @ rotation) - 1) * 0.5
                    for rotation in base_from_boards[:, :3, :3]
                ],
                -1,
                1,
            )
        )
    )
    return {
        "translation_rms_mm": float(
            np.sqrt(np.mean(np.square(translation_errors_mm)))
        ),
        "translation_median_mm": float(np.median(translation_errors_mm)),
        "translation_max_mm": float(np.max(translation_errors_mm)),
        "rotation_rms_deg": float(
            np.sqrt(np.mean(np.square(rotation_errors_deg)))
        ),
        "rotation_median_deg": float(np.median(rotation_errors_deg)),
        "rotation_max_deg": float(np.max(rotation_errors_deg)),
    }


def main():
    parser = argparse.ArgumentParser(description="Solve eye-in-hand AX=XB calibration.")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--max-translation-rms-mm", type=float, default=5.0)
    parser.add_argument("--max-rotation-rms-deg", type=float, default=1.0)
    parser.add_argument(
        "--max-tf-age",
        type=float,
        default=0.1,
        help="Exclude observations whose robot TF differs more than this many seconds",
    )
    args = parser.parse_args()

    session_dir = args.session_dir.expanduser().resolve()
    with (session_dir / "observations.yaml").open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    robot_poses, board_poses, indices, reprojection = redetect_board_poses(
        session_dir, document, args.max_tf_age
    )
    if len(robot_poses) < 10:
        raise RuntimeError(f"Only {len(robot_poses)} usable observations")

    results = {}
    for name, method in METHODS.items():
        rotation, translation = cv2.calibrateHandEye(
            [pose[:3, :3] for pose in robot_poses],
            [pose[:3, 3] for pose in robot_poses],
            [pose[:3, :3] for pose in board_poses],
            [pose[:3, 3] for pose in board_poses],
            method=method,
        )
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation.ravel()
        score = score_solution(robot_poses, board_poses, transform)
        results[name] = {
            "end_effector_from_camera": transform_result(transform),
            "validation": score,
        }

    ranking = sorted(
        results,
        key=lambda name: (
            results[name]["validation"]["translation_rms_mm"]
            / args.max_translation_rms_mm
            + results[name]["validation"]["rotation_rms_deg"]
            / args.max_rotation_rms_deg
        ),
    )
    accepted_methods = [
        name
        for name in ranking
        if results[name]["validation"]["translation_rms_mm"]
        <= args.max_translation_rms_mm
        and results[name]["validation"]["rotation_rms_deg"]
        <= args.max_rotation_rms_deg
    ]
    best = accepted_methods[0] if accepted_methods else ranking[0]
    best_score = results[best]["validation"]
    accepted = bool(accepted_methods)
    output = {
        "accepted": accepted,
        "best_method": best,
        "accepted_methods": accepted_methods,
        "acceptance_limits": {
            "translation_rms_mm": args.max_translation_rms_mm,
            "rotation_rms_deg": args.max_rotation_rms_deg,
        },
        "usable_observations": len(robot_poses),
        "maximum_tf_age_seconds": args.max_tf_age,
        "observation_indices": indices,
        "reprojection_rmse_px": {
            "median": float(np.median(reprojection)),
            "maximum": float(np.max(reprojection)),
        },
        "ranking": ranking,
        "methods": results,
    }
    output_path = session_dir / "handeye_solution.yaml"
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(output, stream, sort_keys=False)

    print(f"Usable observations: {len(robot_poses)}")
    for name in ranking:
        score = results[name]["validation"]
        print(
            f"{name:10s} translation RMS={score['translation_rms_mm']:.2f} mm, "
            f"rotation RMS={score['rotation_rms_deg']:.2f} deg"
        )
    print(f"Best: {best}; accepted={accepted}")
    print(f"Saved {output_path}")
    if not accepted:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
