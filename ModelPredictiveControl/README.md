# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model
The vehicle model used is based upon the following states:

- X and Y position
- Orientation (Psi)
- Velocity
- Cross Track Error
- Orientation (Psi) Error

The cross track error is calculated as the current distance of the car from the center of the road, which is provided as desired waypoints. The orientation error is calculated by differencing the desired orientation, again calculated with the waypoints, and the current orientation. 

The outputs for the model are throttle and steering angle. Throttle is output between -1 and 1, which are full reverse and full forward respectively. Steering is bound between -25 and 25 degrees, converted to radians. Once output by the model, this value is normalized to be between -1 and 1. 

To solve the equations for the optimal (lowest cost) path, the model uses an optimization algorithm provided by the library Ipopt. The cost function is defined in `MPC.cpp`, and is a combitation of the current:

- Cross Track Error
- Orientation Error
- Velocity Error
- Magnitude of Actuation's
- Magnitude of Sequential Actuation's

The combination of these costs, when minimized by Ipopt, provide a trajectory that should be aligned with the center of the track with very few "jerky" actuations. To fine-tune the costs, all of the elements are multiplied by a constant to get the right proportions of each. These values were tuned by hand and are available starting at line 44 in `MPC.cpp`.

Once the initial solution is found by Ipopt, this solution is iteratively projected forward in time using the current actuations.

## Timestep Length and Elapsed Duration

My model uses a timestep of 0.1 seconds and a projection of 8 steps. When I tried to increase the timestep, the model tended to react slowly to the sharp turn after the bridge. When I tried to decrease the timestep, the model tended to get a bit "jerky". Increasing the projection steps had a similar effect to increasing the timesteps where it would react slowly to straightening out after the sharp turn. 

## Latency
   
To simulate real world conditions, a 100ms latency is artificially introduced into the actuations. To deal with this, I projected the actual position of the vehicle forward 110ms using the following equations:

- px += v * cos(psi) * dt
- py += v * sin(psi) * dt

The additional 10ms of latency is added in to account for the processing time of the algorithm. This could be calculated on the fly in the future to produce more accurate results.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
