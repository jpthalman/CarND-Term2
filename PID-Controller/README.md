# CarND-Controls-PID
In this project, I created a PID controller which minimized the Cross-Track Error of a moving car by adjusting the steering angle. The controller is robust to disturbances (turns) up to ~55mph.

---

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

## Tuning Strategy

To tune the controler, I followed the below procedure by hand:

-Set all gains to zero.
-Increase the P gain until the response to a disturbance is steady oscillation.
-Increase the D gain until the the oscillations go away (i.e. it's critically damped).
-Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
-Set P and D to the last stable values.
-Increase the I gain until it brings you to the setpoint with the number of oscillations desired

The entire process took about 2 hours. In the future, I would like to create a training method that will automate this process. The final values that I came up with are as follows:

- P: 0.1
- I: 0.0009
- D: 0.9

I made the P term as large as it could be without affecting the stability of the controler. Too large of a P term would make the controler extremely sensitive to disturbances, and likely cause it to occillate to divergence. I then increased the D term until the occilations from the P term were dampened within a reasonable amount of time. Finally, I increased the I term until the bias in the steering angle was accounted for. 

A very good illustration of the effects of the terms in the PID controller is here:
https://upload.wikimedia.org/wikipedia/commons/3/33/PID_Compensation_Animated.gif 


