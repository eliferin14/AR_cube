# AR Cube
The goal of the project is to use a chessboard as the base plane for a AR cube.

The project consists of two parts:
- camera calibration: use a set of pictures of the chessboard to compute the camera intrinsic parameters
- online AR representation of a 3 dimensional cube: acquire images from a live feed and draw a cube in the correct position, with the correct perspective

## Camera calibration
The camera calibration process is used to find the camera intrinsic parameters, that can later be used to compensate the distortion applied by the lens.

In particular we observe a set of (15) representation of a known object (in my case a chessboard) to define the camera matrix and the distortion parameters. Such variables are then stored in a file, that is used by the other program to get better results