# boris_vision

The program uses Realsense T265 IMU to choose which laserscan is being outputted. 
If the sensor is out of level by a certain degree, RPLIDAR A3 data is replaced by Realsense D435 depth map converted to laserscan. 
Height of this 2D scan can be adjusted.
