#!/usr/bin/env python3

"""
ArUco Marker Pose Estimator (Standalone, NO ROS)

This program:
  - Reads your webcam
  - Loads camera calibration parameters (K and D)
  - Detects ArUco markers
  - Computes 3D pose (translation + rotation)
  - Draws axis and marker box
  - Prints pose to terminal
"""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import sys

# -----------------------------
# USER PARAMETERS
# -----------------------------

aruco_dictionary_name = "DICT_ARUCO_ORIGINAL"   # Type of marker used
aruco_marker_side_length = 0.0785               # Marker size in METERS

# Your calibration file (must contain K and D matrices)
camera_calibration_parameters_filename = "calibration_chessboard.yaml"


# -----------------------------
# HELPER FUNCTIONS
# -----------------------------

def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to Euler roll, pitch, yaw (in radians)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


# -----------------------------
# MAIN PROGRAM
# -----------------------------

def main():

    # -----------------------------
    # Validate dictionary
    # -----------------------------
    ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
    }

    if aruco_dictionary_name not in ARUCO_DICT:
        print(f"❌ Invalid ArUco dictionary: {aruco_dictionary_name}")
        sys.exit(1)

    # -----------------------------
    # Load calibration file
    # -----------------------------
    cv_file = cv2.FileStorage(camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode("K").mat()
    dst = cv_file.getNode("D").mat()
    cv_file.release()

    if mtx is None or dst is None:
        print("❌ Failed to load calibration file. Missing K or D.")
        sys.exit(1)

    print("✅ Loaded calibration file:")
    print("K =\n", mtx)
    print("D =\n", dst)

    # -----------------------------
    # Load ArUco dictionary
    # -----------------------------
    print(f"[INFO] Detecting '{aruco_dictionary_name}' markers...")
    dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])
    parameters = cv2.aruco.DetectorParameters_create()

    # -----------------------------
    # Initialize webcam
    # -----------------------------
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Could not open webcam.")
        sys.exit(1)

    print("🎥 Webcam started. Press 'q' to exit.")

    # -----------------------------
    # Main loop
    # -----------------------------
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Frame capture failed.")
            continue

        # Detect markers (NO cameraMatrix here → needed for OpenCV 4.6 compatibility)
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            frame, dictionary, parameters=parameters
        )

        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)

            # Pose estimation
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                aruco_marker_side_length,
                mtx,
                dst
            )

            for i, marker_id in enumerate(marker_ids):
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]

                # Rotation → quaternion → euler
                rotation_matrix = cv2.Rodrigues(rvec)[0]
                quat = R.from_matrix(rotation_matrix).as_quat()
                roll, pitch, yaw = euler_from_quaternion(*quat)

                print("---- Marker ID:", marker_id[0], " ----")
                print("X:", tvec[0])
                print("Y:", tvec[1])
                print("Z:", tvec[2])
                print("Roll:", math.degrees(roll))
                print("Pitch:", math.degrees(pitch))
                print("Yaw:", math.degrees(yaw))

                # Draw axis
                #cv2.aruco.drawAxis(frame, mtx, dst, rvec, tvec, 0.05)
                cv2.drawFrameAxes(frame, mtx, dst, rvec, tvec, 0.05)



        # Show frame
        cv2.imshow("ArUco Pose Estimation", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # -----------------------------
    # Cleanup
    # -----------------------------
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
