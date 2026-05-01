#!/usr/bin/env python3

import cv2
import numpy as np

# Change this if needed
CAMERA_PATH = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"

CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 15

def marker_center(corners):
    pts = np.asarray(corners).reshape(-1, 2)
    cx = float(np.mean(pts[:, 0]))
    cy = float(np.mean(pts[:, 1]))
    return cx, cy

def main():
    cap = cv2.VideoCapture(CAMERA_PATH, cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)

    if not cap.isOpened():
        print("ERROR: Could not open camera")
        return

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    print("Camera preview started.")
    print("Press 'q' to quit.")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Failed to read frame")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        frame_h, frame_w = frame.shape[:2]
        frame_cx = frame_w / 2.0
        frame_cy = frame_h / 2.0

        # draw image center
        cv2.circle(frame, (int(frame_cx), int(frame_cy)), 5, (255, 0, 0), -1)
        cv2.line(frame, (int(frame_cx), 0), (int(frame_cx), frame_h), (255, 0, 0), 1)
        cv2.line(frame, (0, int(frame_cy)), (frame_w, int(frame_cy)), (255, 0, 0), 1)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                cx, cy = marker_center(corners[i])

                err_x = cx - frame_cx
                err_y = cy - frame_cy

                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)

                text = f"ID:{marker_id} cx:{cx:.1f} cy:{cy:.1f} err_x:{err_x:.1f} err_y:{err_y:.1f}"
                cv2.putText(
                    frame,
                    text,
                    (int(cx) + 10, int(cy) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                print(text)

        cv2.imshow("ArUco Camera Preview", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()