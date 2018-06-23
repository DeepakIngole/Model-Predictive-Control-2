# Model-Predictive-Control 

## Overview 
This project implements a Model Predictive Controller (MPC) in C++ to maneuver a vehicle around the track in a simulator. The simulator will provide the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle. 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

## Build Instructions 
1. Clone the Model-Predictive-Control git repository
    ```  
    $ git clone https://github.com/jfoshea/Model-Predictive-Control.git
    ```
2. This project involves the Term 2 Simulator which can be downloaded here [link](https://github.com/udacity/self-driving-car-sim/releases)

3. Build the project using cmake or using the scipts below 
    ```  
    $ ./clean.sh 
    $ ./build.sh 
    ```
4. Run the model predictive controller 
    1. Launch the simulator and select MPC Controller 
    2. Change to build directory
    ```  
    $ cd build 
    ```
    2. Run the model predictive controller
    ```  
    $ ./mpc
    ```

