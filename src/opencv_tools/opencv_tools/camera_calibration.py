#!/usr/bin/env python3
"""
Camera Calibration Script (OpenCV)

This script:
  - Detects a chessboard pattern in a set of images
  - Computes camera matrix and distortion coefficients
  - Saves them to a YAML file (e.g. calibration_chessboard.yaml)

Usage examples:

  # simplest: use .jpg images in current folder, default pattern (10x7 squares)
  python3 camera_calibration.py

  # specify images directory and output file
  python3 camera_calibration.py --images_dir ./calib_images \
                                --output calibration_chessboard.yaml

  # specify chessboard geometry and square size (meters)
  python3 camera_calibration.py --squares_x 10 --squares_y 7 --square_size 0.025
"""

from __future__ import print_function

import os
import glob
import sys
import argparse

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Camera calibration using chessboard images."
    )

    parser.add_argument(
        "--images_dir",
        type=str,
        default=".",
        help="Directory containing calibration images (default: current directory).",
    )

    parser.add_argument(
        "--squares_x",
        type=int,
        default=10,
        help="Number of chessboard squares along X (horizontal, corners = squares_x - 1).",
    )

    parser.add_argument(
        "--squares_y",
        type=int,
        default=7,
        help="Number of chessboard squares along Y (vertical, corners = squares_y - 1).",
    )

    parser.add_argument(
        "--square_size",
        type=float,
        default=0.025,
        help="Size of one chessboard square in meters (default: 0.025).",
    )

    parser.add_argument(
        "--output",
        type=str,
        default="calibration_chessboard.yaml",
        help="Path to output YAML file (default: calibration_chessboard.yaml).",
    )

    parser.add_argument(
        "--no_view",
        action="store_true",
        help="Disable visualization of detected corners.",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    # Chessboard configuration
    squares_x = args.squares_x      # total number of squares along X
    squares_y = args.squares_y      # total number of squares along Y
    nX = squares_x - 1              # number of inner corners along X
    nY = squares_y - 1              # number of inner corners along Y
    square_size = args.square_size  # meters

    print("=== Camera Calibration ===")
    print(f" Images directory : {args.images_dir}")
    print(f" Squares (X x Y)  : {squares_x} x {squares_y}")
    print(f" Inner corners    : {nX} x {nY}")
    print(f" Square size      : {square_size} m")
    print(f" Output YAML      : {args.output}")
    print("======================================")

    # Prepare object points for one chessboard image:
    # (0,0,0), (1,0,0), ..., (nX-1, nY-1, 0) scaled by square_size
    objp = np.zeros((nX * nY, 3), np.float32)
    # IMPORTANT: mgrid order -> (cols, rows) = (nX, nY)
    objp[:, :2] = np.mgrid[0:nX, 0:nY].T.reshape(-1, 2)
    objp = objp * square_size

    # Arrays to store object points and image points from all images.
    object_points = []  # 3D points in real world space
    image_points = []   # 2D points in image plane.

    # Find images
    pattern_jpg = os.path.join(args.images_dir, "*.jpg")
    pattern_png = os.path.join(args.images_dir, "*.png")

    images = sorted(glob.glob(pattern_jpg) + glob.glob(pattern_png))

    if not images:
        print("❌ No images found in directory:", args.images_dir)
        print("   Expected .jpg or .png files.")
        sys.exit(1)

    print(f"Found {len(images)} image(s). Starting detection...\n")

    img_size = None  # Will hold (width, height) from a valid image

    # Termination criteria for cornerSubPix
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    for idx, image_file in enumerate(images):
        print(f"[{idx+1}/{len(images)}] Processing: {image_file}")

        image = cv2.imread(image_file)
        if image is None:
            print("   ⚠️ Could not read image, skipping.")
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Set image size from first valid image
        if img_size is None:
            img_size = gray.shape[::-1]  # (width, height)

        # NOTE: pattern size is (number of inner corners per row, per column)
        # Here: nX (horizontal) x nY (vertical)
        success, corners = cv2.findChessboardCorners(gray, (nX, nY), None)

        if not success:
            print("   ❌ Chessboard NOT found in this image.")
            continue

        print("   ✅ Chessboard found.")

        # Refine corner locations
        corners_refined = cv2.cornerSubPix(
            gray,
            corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=criteria,
        )

        object_points.append(objp)
        image_points.append(corners_refined)

        # Draw and optionally show corners
        cv2.drawChessboardCorners(image, (nX, nY), corners_refined, success)

        if not args.no_view:
            cv2.imshow("Calibration - Detected Corners", image)
            cv2.waitKey(500)  # show for 0.5s per image

    if not args.no_view:
        cv2.destroyAllWindows()

    # Check we detected at least one board
    if len(image_points) == 0:
        print("\n❌ No chessboard corners detected in ANY image.")
        print("   Please check:")
        print("    - Chessboard pattern size (squares_x, squares_y)")
        print("    - Image quality and lighting")
        print("    - That the full board is visible in the images")
        sys.exit(1)

    if img_size is None:
        print("\n❌ Could not determine image size. Aborting.")
        sys.exit(1)

    print("\n=== Running calibration with "
          f"{len(image_points)} valid image(s) ===")

    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        img_size,
        None,
        None,
    )

    print("\nCalibration RMS reprojection error:", ret)
    print("\nCamera matrix (K):")
    print(mtx)
    print("\nDistortion coefficients (D):")
    print(dist.ravel())

    # Save to YAML file
    output_path = args.output
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    cv_file = cv2.FileStorage(output_path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    cv_file.release()

    print(f"\n✅ Calibration saved to: {output_path}")
    print("   (keys: 'K' for cameraMatrix, 'D' for distCoeffs)")


if __name__ == "__main__":
    print(__doc__)
    main()
