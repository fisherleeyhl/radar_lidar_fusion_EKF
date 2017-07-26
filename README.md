# Radar and Lidar fusion using an extended Kalman Filter

This project is to use an extended Kalman Filter to fuse the radar and Lidar sensor measurements using C/C++. 

## Repo content

* src contains the project source files.
  (a) main.cpp entry point of the project, loads the data, specifies motion and observation models, calls kalman filter to fuse the radar and lidar measurements.
  (b) fusion_radar_lidar.cpp initializes the filter, calls the prediction and update of the kalman filter.
  (c) kalman_filter.cpp defines initialization, prediction, and update functions for the Kalman Filter.
  (d) tools.cpp computes RMSE and Jacobian matrix.
* include contains the project header files.
* data contains two example input files.
* docs contains a documentation file specifying the input and ouput data file format.
* result contains the generated result files

## How to run the code
For the project, run the following commands in Linux terminal.
* mkdir build 
* cd build 
* cmake ..
* make
* ./radar_lidar_fusion ../data/lidar_radar_input1.txt ../result/result1.txt
