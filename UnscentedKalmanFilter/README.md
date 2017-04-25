# Unscented Kalman Filter in C++

In this project, I created an Unscented Kalman Filter with the purpose of localizing a moving pedestrian accurately by fusing Radar and Lidar data. This type of bayesian filter is very good at tracking non-linear movement, and improved significantly on the Extended Kalman Filter's result.

## Dependencies

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4

## Basic Build Instructions

- Clone this repo
- Make a build directory: `mkdir build && cd build`
- Compile: `cmake .. && make`
- Run it: `./UnscentedKalmanFilter path/to/input_data.txt path/to/output.txt`\
- Some example data and outputs are located in `data/`

## Results

<div align="center">
	<img src="https://github.com/jpthalman/CarND-Term2/blob/master/UnscentedKalmanFilter/images/results_plot.png">
</div>

When ran against `data/sample-laser-radar-measurement-data-1.txt`, the following RMSE's are obtained:

- P<sub>x</sub> : 0.0698449
- P<sub>y</sub> : 0.0861106
- V<sub>x</sub> : 0.3442510
- V<sub>y</sub> : 0.3255100
