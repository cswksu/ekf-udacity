# Extended Kalman Filter Project

## Intro

Self-Driving Car Engineer Nanodegree Program

In this project I utilized a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. RMSE values were obtained that were lower than the tolerance outlined in the project rubric. 

## Rubric Response
### Compiling
*Your code should compile.*

The code I've create compiles using cmake and make. No manual changes were made to CMakeLists.txt.

### Accuracy
*px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1.*

See below file for proof of RMSE:

![rmse proof](https://github.com/cswksu/ekf-udacity/blob/master/photos/RMSE%20proof.png?raw=true)

RMSE is [0.0985, 0.0850, 0.4508, 0.4317].

### Follows the Correct Algorithm
*Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.*

The algorithm used follows the algorithm shown in the lessons very closely. The first measurement initializes the state and covariance matrices. For subsequent measurements, a prediction occurs based on constant acceleration dynamics. Next, the appropriate Kalman filter (conventional for LIDAR, extended for RADAR) is used to update the state and covariance matrices.

*Your Kalman Filter algorithm handles the first measurements appropriately.*

The state matrix and covariances matrices are initialized with the first measurement. For LIDAR measurements, only the x and y are entered into the state matrix. For RADAR measurements, rho-dot is used to get an estimate of x- and y-velocities. The component of rho-dot along the x- and y-axes are put into the state matrices. This neglects any angular component, but is better than setting initial estimates to zero.

*Your Kalman Filter algorithm first predicts then updates.*

Yes, the algorithm first predicts, then updates using the approptriate function, depending on measurement type.

*Your Kalman Filter can handle radar and lidar measurements.*

Yes, LIDAR is handled by a conventional Kalman filter and RADAR is handled by the EKF.

### Code Efficiency

*Your algorithm should avoid unnecessary calculations.*

An effort was made to reduce unnecessary computations, such as common expressions in the Q and Hj matrices.

## Further improvements

The code could be further improved by optimizing the RMSE function. Currently, for every timestep, each estimate and ground truth entry is used in calculating the latest RMSE. Only the latest residuals and the previous RMSE need to be considered. 

## How to use this program

### Key Dependencies and Build Instructions

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

### Other Important Dependencies

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

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
