import numpy as np
import cv2
import time
from pytransform3d import transformations as tf
from pytransform3d import rotations as rt

# Function to draw two images side by side
def concatenate_images(image1, image2):
    """
    Concatenates two images side by side.
    
    Args:
        image1: The first input image.
        image2: The second input image.
    
    Returns:
        concatenated_image: The concatenated image.
    """
    # Assuming images have the same dimensions
    height = image1.shape[0]
    width = image1.shape[1] + image2.shape[1]

    # Create a new blank image with combined width
    concatenated_image = np.zeros((height, width, 3), dtype=np.uint8)

    # Copy the first image to the left side
    concatenated_image[:, :image1.shape[1]] = image1

    # Copy the second image to the right side
    concatenated_image[:, image1.shape[1]:] = image2

    return concatenated_image

# Load the intrinsic parameters
calibration_data = np.load('calibration_params.npy', allow_pickle=True).item()
camera_matrix = calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']

# Compute the undistort map once for all. We assume to be working with (640,480) images
image_width = 640
image_height = 480
und_map_x, und_map_y = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, None, (image_width, image_height), cv2.CV_32FC1)

# Define the chessboard corners' 3D coordinates
def define_chessboard_3D_coordinates(pattern_size):
    rows = pattern_size[0]
    cols = pattern_size[1]
    coord = np.zeros((rows*cols,3), np.float32)
    coord[:,:2] = np.mgrid[0:rows,0:cols].T.reshape(-1,2)
    return coord

# Object points: we are using a 6x8 squares chessboard -> 5x7 corners
pattern_size = (7,5)
object_points = define_chessboard_3D_coordinates(pattern_size)

# Criteria for cornerSubPix
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Function to map a point in the object frame to the corresponding pixel coordinates
def project_point(point, camera_matrix, rvec, tvec):
    # Calculate the rotation matrix from the rotation vector
    rot_matrix, _ = cv2.Rodrigues(rvec)

    # Define the transformation matrix
    T = np.vstack( [np.hstack([rot_matrix, tvec]), [0, 0, 0, 1]])
    #W = np.hstack([rot_matrix, tvec])

    # Transform the point to the camera frame (after converting to homogeneous)
    point_homogeneous = np.vstack([point.reshape(-1,1), 1])
    transformed_point = T @ point_homogeneous
    
    # Project the point onto the image plane
    P = np.hstack([camera_matrix, np.zeros((3,1))])
    point_image_plane_homogeneous = P @ transformed_point
    point_image_plane_homogeneous /= point_image_plane_homogeneous[2]
    point_image_plane = point_image_plane_homogeneous[:-1]

    return (point_image_plane[0][0], point_image_plane[1][0])

def project_array_of_points(points, camera_matrix, rvec, tvec):
    projected_points = np.zeros((2, points.shape[1]))
    for i in range(0, points.shape[1]):
        point = points[:,i]
        projected_point = project_point(point, camera_matrix, rvec, tvec)
        projected_points[:,i] = projected_point
    return projected_points

# Function to draw the object reference frame in the image
def draw_object_frame(points, image, camera_matrix, rvec, tvec, line_width):
    # Compute pixel coordinates of the frame points
    projected_frame_points = project_array_of_points(points, camera_matrix, rvec, tvec)
    projected_frame_points_pixels = np.round(projected_frame_points).astype(int)

    # Draw the frame vectors
    cv2.line(image, projected_frame_points_pixels[:,0], projected_frame_points_pixels[:,1], (0,0,255), line_width)
    cv2.line(image, projected_frame_points_pixels[:,0], projected_frame_points_pixels[:,2], (0,255,0), line_width)
    cv2.line(image, projected_frame_points_pixels[:,0], projected_frame_points_pixels[:,3], (255,0,0), line_width)
    return image

# Notable reference frames
frame_points = np.hstack([ np.zeros((3,1)), np.eye(3) ])

# Calculate transformation matrix from Euler angles and translation vector
def euler_transformation_matrix(euler_angles, translation):
    rotation_matrix = rt.active_matrix_from_intrinsic_euler_zyz(euler_angles)
    T = tf.transform_from(rotation_matrix, translation)
    return T

def apply_transformation(transformation, points):
    points_homogeneous = np.vstack([points, np.ones(points.shape[1])])
    transformed_points_homogeneous = transformation @ points_homogeneous
    transformed_points = transformed_points_homogeneous[:-1]
    #print(transformed_points)
    return transformed_points

# Transformation from the corner rf to the center rf
euler_angles = [0,np.pi,np.pi]
translation = np.array([3,2,0])
corner_to_center_transformation = euler_transformation_matrix(euler_angles, translation)
center_frame_points = apply_transformation(corner_to_center_transformation, frame_points)

# Cube in the center
cube_points = np.vstack([[1,1,0],[-1,1,0],[-1,-1,0],[1,-1,0],[1,1,2],[-1,1,2],[-1,-1,2],[1,-1,2]]).T
cube_points_transformed = apply_transformation(corner_to_center_transformation, cube_points)
cube_angular_velocity = 0.5

def draw_cube(points, image, camera_matrix, rvec, tvec, color, line_width):
    # Project points to pixels
    points = project_array_of_points(points, camera_matrix, rvec, tvec)
    points = np.round(points).astype(int)
    # Bottom square
    cv2.line(image, points[:,0], points[:,1], color, line_width)
    cv2.line(image, points[:,1], points[:,2], (0,255,0), line_width)
    cv2.line(image, points[:,2], points[:,3], color, line_width)
    cv2.line(image, points[:,3], points[:,0], color, line_width)
    # Top square
    cv2.line(image, points[:,4], points[:,5], color, line_width)
    cv2.line(image, points[:,5], points[:,6], color, line_width)
    cv2.line(image, points[:,6], points[:,7], color, line_width)
    cv2.line(image, points[:,7], points[:,4], color, line_width)
    # Sides
    cv2.line(image, points[:,0], points[:,4], color, line_width)
    cv2.line(image, points[:,1], points[:,5], color, line_width)
    cv2.line(image, points[:,2], points[:,6], (255,0,0), line_width)
    cv2.line(image, points[:,3], points[:,7], color, line_width)

    return image

#######################
def main():

    # Open the video capture
    camera = cv2.VideoCapture('/dev/video0')

    # Check if the camera was opened successfully
    if not camera.isOpened():
        print("Error: Couldn't open camera.")
        return
    
    # Create a window and set the mouse callback function
    cv2.namedWindow('Frame')

    # FPS indicator
    fps = 0

    # Start of the experiment
    time_0 = time.time()

    while True:
        # Start the time
        start_time = time.time()

        # Capture frame
        ret, distorted_image = camera.read()

        # Check if the frame was captured successfully
        if not ret:
            print("Error: Couldn't capture frame.")
            break

        # Apply undistortion
        #image = cv2.remap(distorted_image, und_map_x, und_map_y, cv2.INTER_LINEAR)
        #image = cv2.undistort(distorted_image, camera_matrix, dist_coeffs)
        image = distorted_image.copy()

        # Find the chessboard corners in the image
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_image, pattern_size, None)
        if ret:
            corners = cv2.cornerSubPix(gray_image, corners.astype(np.float32), (11,11), (-1,-1), criteria)
            #image = cv2.drawChessboardCorners(image, pattern_size, corners, ret)

            # Compute extrinsic parameters
            ret, rvec, tvec = cv2.solvePnP(object_points, corners, camera_matrix, dist_coeffs)

            # Draw reference frame
            image = draw_object_frame(frame_points, image, camera_matrix, rvec, tvec, 2)
            image = draw_object_frame(center_frame_points, image, camera_matrix, rvec, tvec, 2)

            # Calculate the rotation reference frame
            euler_angles_cube = np.array([cube_angular_velocity*start_time, 0, 0])
            center_to_cube_transformation = euler_transformation_matrix(euler_angles_cube, np.array([0,0,0]))
            #print(center_to_cube_transformation)
            cube_coord_center_frame = apply_transformation(center_to_cube_transformation, cube_points)
            cube_coord_corner_frame = apply_transformation(corner_to_center_transformation, cube_coord_center_frame)
            image = draw_cube(cube_coord_corner_frame, image, camera_matrix, rvec, tvec, (0,0,255), 4)

        # Add text
        font_scale = 0.5
        text_height = 20
        cv2.putText(image, f"FPS: {fps:.2f}", (10, 1*text_height), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), 2)
        if not ret: cv2.putText(image, f"Board not detected!", (10, 2*text_height), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0,255), 2)

        output = concatenate_images(distorted_image, image)
        cv2.imshow('Frame', image)

        # Check for key press; if 'q' is pressed, exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        loop_time = time.time() - start_time
        fps = fps + 0.2*(1/loop_time - fps)


    # Release the capture device and close all OpenCV windows
    camera.release()
    cv2.destroyAllWindows()

main()