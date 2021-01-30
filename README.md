[//]: # (Image References)

[simulator]: ./output_images/simulator.png "Simulator"
[dataset1]: ./output_images/dataset1.png "Dataset 1"
[dataset2]: ./output_images/dataset2.png "Dataset 2"
[simulation]: ./output_images/simulation.gif "Simulation"
# **Extended Kalman Filter** 

## Report

---

**Extended Kalman Filter Project**

In this project, an extended kalman filter is implemented in C++ to estimate the state of a moving object with noisy RADAR and LiDAR measurements. A [simulator](https://github.com/udacity/self-driving-car-sim/releases) provided by Udacity is used to generate these noisy data.

The key metrics are RMSE values for both position and velocity of the tracked object.


## Project Strutcture
My project includes the following files:
* [`FusionEKF.cpp`](https://github.com/mhusseinsh/CarND-Extended-Kalman-Filter-Project/blob/master/src/FusionEKF.cpp) initializes the filter, calls the predict function, calls the update function
* [`FusionEKF.h`](https://github.com/mhusseinsh/CarND-Extended-Kalman-Filter-Project/blob/master/src/FusionEKF.h) header file where definitions are done
* [`json.hpp`](https://github.com/mhusseinsh/CarND-Extended-Kalman-Filter-Project/blob/master/src/json.hpp) basic JSON functions
* [`kalman_filter.cpp`](https://github.com/mhusseinsh/CarND-Extended-Kalman-Filter-Project/blob/master/src/kalman_filter.cpp) defines the predict function, the update function for lidar, and the update function for radar  
* [`kalman_filter.h`](https://github.com/mhusseinsh/CarND-Extended-Kalman-Filter-Project/blob/master/src/kalman_filter.h) header file where definitions are done
* [`main.cpp`](https://github.com/mhusseinsh/CarND-Extended-Kalman-Filter-Project/blob/master/src/main.cpp) communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
* [`measurement_package.h`](https://github.com/mhusseinsh/CarND-Extended-Kalman-Filter-Project/blob/master/src/measurement_package.h) definition of measurement_package class
* [`tools.cpp`](https://github.com/mhusseinsh/CarND-Extended-Kalman-Filter-Project/blob/master/src/tools.cpp) function to calculate RMSE and the Jacobian matrix
* [`tools.h`](https://github.com/mhusseinsh/CarND-Extended-Kalman-Filter-Project/blob/master/src/tools.h) header file where definitions are done 
* [`README.md`](https://github.com/mhusseinsh/CarND-Extended-Kalman-Filter-Project/blob/master/README.md) or writeup_report.md or writeup_report.pdf summarizing the results


## Prerequisites

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Required tools are:
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

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

## Build and Install
Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.
```sh
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF
```
## Running the project

To run the EKF after building and installing: `./ExtendedKF`. The output will be as below:
```sh
Listening to port 4567
```
Here the EKF will be waiting for the simulator to launch and start, and once it started, the below message will appear in the console.
```sh
Connected!!!
```
Initially, the simulator looks like this

![alt text][simulator]

The simulator provides two datasets to choose from

* **Dataset 1**: The car begins moving to the right forming an infinity shape, and the order of the measurements begin with a RADAR measurement sent first followed by LiDAR measurement and so on.
* **Dataset 2**: The car begins moving to the left forming an infinity shape, and the order of the measurements begin with a LiDar measurement sent first followed by RADAR measurement and so on.

## Protocol
Below is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

*INPUT*: values provided by the simulator to the C++ program:
```
["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)
```

*OUTPUT*: values provided by the C++ program to the simulator:
```
["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]
```

## Results Evaluation

Based on the defined [Rubric Points](https://review.udacity.com/#!/rubrics/748/view) the RMSE value should <= [0.11, 0.11, 0.52, 0.52] in px, py, vx, vy respectively.

The results for the two datasets are shown below:

| RMSE | Dataset 1 | Dataset 2 |
|------|-----------|-----------|
| px  |  0.0973   |  0.0740   |
| py  |  0.0855   |  0.0963   |
| vx  |  0.4513   |  0.4463   |
| vy  |  0.4399   |  0.4752   |

### Dataset 1:

![alt text][dataset1]

### Dataset 2:

![alt text][dataset2]

And here is a run example:

![alt text][simulation]

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. The above simulation shows what the simulator looks like when a c++ script is using its Kalman filter to track the object. The simulator provides the script the measured data (either lidar or radar), and the script feeds back the measured estimation marker, and RMSE values from its Kalman filter.

