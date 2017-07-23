# Radar and Lidar fusion using an extended Kalman Filter

This project is to use an extended Kalman Filter to fuse the radar and Lidar sensor measurements using C/C++. 

## Repo content

* src contains the project source files.
  (a) main.cpp entry point of the project, reads the data, calls kalman filter to fuse the radar and lidar measurements.
  (b) kalman_filter.cpp defines functions for Kalman Filter.
* include contains the project header files.
  (a) kalman_filter.h declares functions for Kalman Filter.
* data contains the input files and the generated result files.
* docs contains some documentation files.

## How to run the code
For each project, run the following commands in Linux terminal.
* project1:
(a) mkdir build 
(b) cd build 
(c) cmake ..
(d) make
(e) ./radar_lidar_fusion ../data/lidar_radar_input1.txt ../data/result1.txt
