#https://docs.opencv.org/4.9.0/df/d4a/tutorial_charuco_detection.html

import numpy as np
import cv2
import time
from pytransform3d import transformations as tf
from pytransform3d import rotations as rt

# Load the intrinsic parameters
calibration_data = np.load('calibration_params.npy', allow_pickle=True).item()
camera_matrix = np.array([ [1,0,480],[0,1,320],[0,0,1]])
dist_coeffs = np.zeros(5)

# ChArUco board
charuco_flag = True
markers_x = 8
markers_y = 6
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard((markers_x,markers_y), 20, 15, dictionary)
board.setLegacyPattern(True)
board_image = board.generateImage((1920,1080))
cv2.imwrite("board.jpg", board_image)

# Parameters of the detector
params = cv2.aruco.DetectorParameters()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

# Define the detector
detector = cv2.aruco.ArucoDetector(dictionary, params)

####################################
cv2.namedWindow("Frame")

# Load the image
image = cv2.imread("../ChArUco_images/charuco1.jpg")

# Convert to grayscale
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect ArUco markers
marker_corners, marker_ids, candidates_rejected = detector.detectMarkers(gray_image)

# If something is detected, go on
if marker_ids is not None:
    # Refine detected markers
    [marker_corners, marker_ids, candidates_rejected, recovered_ids] = detector.refineDetectedMarkers(gray_image, board, marker_corners, marker_ids, candidates_rejected)
    print(recovered_ids)
    # Draw on the image
    image = cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)

objPoints = np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]])
for marker_corner in marker_corners:
    valid, rvec, tvec = cv2.solvePnP(objPoints, marker_corner, camera_matrix, dist_coeffs)
    cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, 1)





# Display result
cv2.imshow("Frame", image)
cv2.waitKey(0)
