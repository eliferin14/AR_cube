# AR Cube
The goal of the project is to use a chessboard as the base plane for a AR cube.

The project consists of two parts:
- camera calibration: use a set of pictures of the chessboard to compute the camera intrinsic parameters
- online AR representation of a 3 dimensional cube: acquire images from a live feed and draw a cube in the correct position, with the correct perspective

## Camera calibration
The camera calibration process is used to find the camera intrinsic parameters, that can later be used to compensate the distortion applied by the lens.

In particular we observe a set of (15) representation of a known object (in my case a chessboard) to define the camera matrix and the distortion parameters. Such variables are then stored in a file, that is used by the other program to get better results

The process is the standard calibration algortihms described in the literature

## Live AR cube

### Undistortion
The first step is to load the intrinsic parameters, defined before.

Using the distortion parameters I calculated the undistortion map (using the dedicated function `initUndistortRectifyMap()` in OpenCV). This map is reasonably assumed to be constant. In the main loop, every time we acquire an image we can undistort it using `remap()`

### Chessboard frame
Knowing the size of the chessboard, I defined the corners coordinates in 3D space as coplanar points with $z=0$

In the main loop, we will use such points to calculate the extrinsic parameters, namely the rotation and translation vectors that describe the relative pose of the camera frame and the chessboard frame

The extrinsic parameters are calculated by solving a MSE problem, using the function `solvePnP()`. From these parameters we can calculate the trasnformation matrix

### Projection of 3D points into the image
Given a point in 3D space, and knowing the intrinsic and extrinsic parameters, we can calculate the pixel coordinates of the corresponding 2D point using a succession of matrix multiplications.

The function that calculates such projection can be described as:
- calculate the transformation matrix between camera frame and the frame used to define the point coordinates (4x4 matrix)
- redefine the point in homogeneous coordinates (4x1 vector)
- calculate the point in camera frame by multiplying it with the transformation matrix (Note: the result is a 4x1 vector)
- calculate the projection matrix by stacking the camera matrix and a column of zeros (3x4 matrix)
- project the point into the image plane (the result is a 3x1 vector -> a 2D point in homogeneous coordinates)
- return to planar coordinates by normalizing the homogeneous coordinates (divide the vector by its last element) and then removing the last element (that should be a 1)
- quantize the 2D coordinates to obtain pixels

This process can be applied to any 3D point we want, as long as we have its coordinates and the reference frame

#### Drawing of a reference frame
We can draw a 3D reference frame in the image by calculating th pojection of the origin and of the 3 unit vectors

I chose to use the standard colors for the 3 vectors (red fo rx, green for y, blue for z)

### Drawing the cube
At this point drawing the cube is just a matter of defining its corners in a suitable reference frame, and then projecting them onto the image.

I wanted the cube to be in the center of the chessboard, so I defined a central reference frame by manually defining a translation vector. Also, to impose the desired orientation (y pointing up, z exiting the plane) I used a triplet of Euler angles to define the rotation matrix

The transformation matrix from the camera frame to this central frame can be obtained by multiplying the camera2chessboard transformation by the chessboard2center transformation 

The cube is designed such that the side is 2cm long (two chessboard squares), and the square base is centered in the central reference frame

[The cube is dynamically calculated as the chessboard moves (video)](https://imgur.com/CVmUvCD)

#### Rotating cube
To add a cool effect, I made the cube rotate

To do that I simply needed to recalculate the transformation between the central frame and the cube frame at each time step, using an evolving angle (constant angular speed)

[The cube rotates aswell (video)](https://imgur.com/ENrEHXJ)

