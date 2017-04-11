# Extended Kalman Filter in C++

In this project, I created an Extended Kalman Filter with the purpose of localizing a moving pedestrian accurately by fusing Radar and Lidar data.

## Dependencies

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4

## Basic Build Instructions

- Clone this repo
- Make a build directory: `mkdir build && cd build`
- Compile: `cmake .. && make`
- Run it: `./ExtendedKalmanFilter path/to/input_data.txt path/to/output.txt`\
- Some example data and outputs are located in `data/`

## Results

When ran against `data/sample-laser-radar-measurement-data-1.txt`, the following RMSE's are obtained:

- P<sub>x</sub> : 0.066145
- P<sub>y</sub> : 0.060362
- V<sub>x</sub> : 0.530336
- V<sub>y</sub> : 0.544382

When ran against `data/sample-laser-radar-measurement-data-2.txt`, the resulting RMSE's are slightly worse:

- P<sub>x</sub> : 0.185925
- P<sub>y</sub> : 0.190263
- V<sub>x</sub> : 0.477258
- V<sub>y</sub> : 0.805505
