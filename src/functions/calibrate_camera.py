import cv2 as cv
import glob
import numpy as np

def calibrate_camera():

    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2) # units don't matter for our purposes
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    
    # List of paths to the calibration images
    images = glob.glob("data/calibration_images/*.png")
    
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (7,6), None)
    
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
    
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
    
            # Draw and display the corners
            cv.drawChessboardCorners(img, (7,6), corners2, ret)
            cv.imshow('img', img)
            cv.waitKey(500)

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    cv.destroyAllWindows()

    return ret, mtx, dist, rvecs, tvecs

    

def main():
    
    ret, mtx, dist, rvecs, tvecs = calibrate_camera()
    print(f"ret:\n{ret}\nmtx:\n{mtx}\ndist:\n{dist}\nrvecs:\n{rvecs}\ntvecs:\n{tvecs}")
    # Once run: copy/paste mtx and dist to camera_matrix and distortion_coeffs in get-image-points-gps
    

if __name__ == "__main__":
    main()