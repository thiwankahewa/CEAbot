"""Shared, ROS-independent hand-eye geometry helpers."""

import cv2
import numpy as np


DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in (
        "DICT_4X4_50",
        "DICT_4X4_100",
        "DICT_5X5_50",
        "DICT_5X5_100",
        "DICT_6X6_50",
        "DICT_6X6_100",
    )
}


def rotation_matrix_to_quaternion(rotation):
    """Return an xyzw quaternion from a proper 3x3 rotation matrix."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    trace = np.trace(rotation)
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2
        return np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    diagonal = np.diag(matrix[:3, :3])
    index = int(np.argmax(diagonal))
    if index == 0:
        scale = np.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
        quaternion = [
            0.25 * scale,
            (matrix[0, 1] + matrix[1, 0]) / scale,
            (matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
        ]
    elif index == 1:
        scale = np.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
        quaternion = [
            (matrix[0, 1] + matrix[1, 0]) / scale,
            0.25 * scale,
            (matrix[1, 2] + matrix[2, 1]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
        ]
    else:
        scale = np.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
        quaternion = [
            (matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[1, 2] + matrix[2, 1]) / scale,
            0.25 * scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        ]
    return np.asarray(quaternion)


def board_outline(squares_x, squares_y, square_length):
    """Sample the perimeter too, since distortion can curve projected edges."""
    width, height = squares_x * square_length, squares_y * square_length
    points = []
    for t in np.linspace(0, 1, 16):
        points.extend(((t * width, 0, 0), (width, t * height, 0),
                       ((1 - t) * width, height, 0), (0, (1 - t) * height, 0)))
    return np.asarray(points, dtype=np.float64)


def board_in_image(camera_from_board, outline, camera_matrix, distortion,
                   width, height, margin):
    points = outline @ camera_from_board[:3, :3].T + camera_from_board[:3, 3]
    if not np.isfinite(points).all() or np.any(points[:, 2] <= 0):
        return False
    rvec, _ = cv2.Rodrigues(camera_from_board[:3, :3])
    pixels, _ = cv2.projectPoints(outline, rvec, camera_from_board[:3, 3],
                                  camera_matrix, distortion)
    pixels = pixels.reshape(-1, 2)
    return bool(np.isfinite(pixels).all()
                and np.all(pixels >= margin)
                and np.all(pixels[:, 0] < width - margin)
                and np.all(pixels[:, 1] < height - margin))


def generate_camera_views(seed_camera, board_center, count=24,
                          tilt_deg=15.0, roll_deg=12.0, distance_variation=0.08):
    """Orbit the observed board center; optical +Z always points at it.

    Alternate two angular radii and distances, with roll variation, to excite
    multiple rotation axes. All poses are expressed in the robot base frame.
    """
    center = np.asarray(board_center, dtype=float)
    outward = seed_camera[:3, 3] - center
    distance = np.linalg.norm(outward)
    if not np.isfinite(distance) or distance <= 0.05:
        raise ValueError("Board is too close to generate calibration views")
    outward /= distance
    right = np.cross(seed_camera[:3, 1], -outward)
    right /= np.linalg.norm(right)
    down = np.cross(-outward, right)
    for i in range(count):
        azimuth = 2 * np.pi * i / count
        tilt = np.radians(tilt_deg) * (0.6 if i % 2 else 1.0)
        radius = distance * (1 + distance_variation * (1 if i % 2 else -1))
        position = center + radius * (
            np.cos(tilt) * outward
            + np.sin(tilt) * (np.cos(azimuth) * right + np.sin(azimuth) * down)
        )
        z_axis = center - position
        z_axis /= np.linalg.norm(z_axis)
        x_axis = np.cross(down, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        roll, _ = cv2.Rodrigues(np.array([0., 0., np.radians(roll_deg) *
                                        ((i % 3) - 1)]))
        pose = np.eye(4)
        pose[:3, :3] = np.column_stack((x_axis, y_axis, z_axis)) @ roll
        pose[:3, 3] = position
        yield pose
