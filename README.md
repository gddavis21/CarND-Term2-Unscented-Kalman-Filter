# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

Utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

---

## Implementation Overview

* Top-level __main()__ function (main.cpp):
  + Create instance of Unscented Kalman Filter algorithm (class UKF), initialized with manufacturer supplied measurement uncertainty values for Lidar/Radar sensors.
  + Parse time-series of Lidar/Radar sensor measurements and ground-truth position/velocity values for a bicycle in motion. 
  + For each measurement:
    - Update bicycle state with UKF algorithm, given current measurement.
    - Query estimated bicycle position/velocity from UKF.
    - Report RMSE of position/velocity estimate relative to ground-truth values.
  + NOTE: main.cpp was (almost) entirely supplied from starter code.

* Class __UKF__ (ukf.h / ukf.cpp)
  + Construction:
    - User supplies radar/lidar sensor uncertainty values.
    - User can also configure UKF instance to ignore radar or lidar measurements.
  + Method __ProcessMeasurement()__ receives Lidar/Radar measurements and updates estimated position/velocity.
    - Initializes state estimate on first measurement.
    - For subsequent measurements, updates state estimate using Unscented Kalman Filter algorithm.
    - New state is predicted from prior state and time delta since last measurement.
    - New state is updated with current radar/lidar measurement.
  + Query methods __GetCurrentPosition()__ and __GetCurrentVelocity()__ to get current position/velocity estimates.

* Class method __Tools::CalculateRMSE()__ (tools.h / tools.cpp) implements the Root-Mean-Square-Error metric for measuring accuracy of the state estimation. 

---

## Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.
