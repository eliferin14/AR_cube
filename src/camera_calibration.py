import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Define the corners of the chessboard in 3D space. All points are coplanar, with z=0
pattern_size = (7,5)
object_points = np.zeros((pattern_size[0]*pattern_size[1],3), np.float32)
object_points[:,:2] = np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Calibration images are stored in another directory
images = glob.glob('../calibration_images/*.jpg')

# Loop for all the images
for fname in images:

    # Load the image and convert to grayscale
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv.findChessboardCorners(gray, pattern_size, None)

    # If found add object points and image points to the corresponding arrays
    if ret == True:
        objpoints.append(object_points)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)    # Refine corners to get better precision
        imgpoints.append(corners2)
        # Draw the corners
        cv.drawChessboardCorners(img, pattern_size, corners2, ret)

    # Show the corners (just to check everything is correct)
    cv.imshow('img', img)
    cv.waitKey(500)
cv.destroyAllWindows()

# Calculate the calibration parameters (here we care only about the instrinsic ones)
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save to file
np.save('calibration_params.npy', {'camera_matrix': mtx, 'dist_coeffs': dist})