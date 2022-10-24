# Visual-acoustic state estimation
Fusion of camera-tag pose, DVL velocity, and IMU data using the multiplicative extended Kalman filter (MEKF).

## Overview
This repo contains source code for visual-acoustic state estimation. A custom error-state Kalman filter is implemented with the ROS interface. It fuses camera-tag pose data (e.g., from AprilTags), DVL measurements, and IMU measurements and outputs the full state of the vehicle, including position, velocity, and attitude.

## Future work
Update readme and make the code more configurable
