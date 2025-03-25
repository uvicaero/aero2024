import cv2 as cv
import glob
import numpy as np
import sys
import os

def calibrate_camera():

    if len(sys.argv) != 2 or sys.argv[1] not in ['noir', 'wide']:
        print("Usage: python3 script.py [noir|wide]")
        sys.exit(1)

    camera_type = sys.argv[1]

    if camera_type == 'noir':
        RESOLUTION = (3280, 2464)
        IMAGE_DIR = 'data/calibration_images/noir'
    elif camera_type == 'wide':
        RESOLUTION = (1280, 720)
        IMAGE_DIR = 'data/calibration_images/wide'

    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    
    objpoints = []
    imgpoints = []
    
    images = glob.glob(f"{IMAGE_DIR}/*.jpg")

    if not images:
        print(f"No images found in {IMAGE_DIR}")
        sys.exit(1)

    found_count = 0
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, corners = cv.findChessboardCorners(gray, (7,6), None)

        if ret:
            found_count += 1
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            cv.drawChessboardCorners(img, (7,6), corners2, ret)
            cv.imshow('img', img)
            cv.waitKey(500)
        else:
            print(f"Chessboard not detected in image {fname}")

    cv.destroyAllWindows()

    if found_count == 0:
        print("No chessboards detected. Calibration failed.")
        sys.exit(1)

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, RESOLUTION, None, None)

    return ret, mtx, dist, rvecs, tvecs

def main():
    ret, mtx, dist, rvecs, tvecs = calibrate_camera()
    print(f"ret:\n{ret}\n\nmtx:\n{mtx}\n\ndist:\n{dist}\n\nrvecs:\n{rvecs}\n\ntvecs:\n{tvecs}")
    # Once run: copy/paste mtx and dist to camera_matrix and distortion_coeffs in get-image-points-gps

if __name__ == "__main__":
    main()
