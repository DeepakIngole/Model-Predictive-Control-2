# **Model-Predictive-Control** 

## 1. Overview 
This project implements a Model Predictive Controller (MPC) in C++ to maneuver a vehicle around the track in a simulator. The websocket server provides reference waypoints (yellow line), and the MPC computes the steering and throttle actuations to drive the car. The goal of a model predictive control is to optimize the control inputs until a low cost vector of control inputs is found. In this case the control inputs are steering angle and throttle.

## 2. MPC Project Description

### 2.1 The Model

MPC uses a model of a system to predict its future behavior, and it solves an optimization problem to select the best control action.  Unlike PID controllers, MPC can handle multi-input multi-output systems that have interactions between their inputs and outputs. However, MPC can simultaneously control all the outputs while taking into account input-output interactions.  MPC can also handle constraints which are important to avoid undesired consequences. MPC has preview capabilities (prediction horizon). If set point changes are known in advance, the controller can better react to those changes and improve its performance. The MPC controller used in this project uses a Kinematic Model. Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more practical in real applications. 

The model consists of state estimator vector and a control vector. The state estimator vector is represented by (px, py, psi, v, cte, epsi). 
  - px:   vehicle x position 
  - py:   vehicle y position 
  - psi:  vehicle angle in radians from x position
  - v:    vehicle velocity
  - cte:  cross track error
  - epsi: orientation error.
 
The control vector is used for navigating the vehicle around the track. The control vector [steering angle, acceleration]

The MPC update equations are as follows:

![](https://latex.codecogs.com/gif.latex?x_%7Bt&plus;1%7D%20%3D%20xt%20&plus;%20vt.cos%28%5Cpsi%20t%29.dt)

![](https://latex.codecogs.com/gif.latex?y_%7Bt&plus;1%7D%20%3D%20yt%20&plus;%20vt.sin%28%5Cpsi%20t%29.dt)

![](https://latex.codecogs.com/gif.latex?%5Cpsi_%7Bt&plus;1%7D%20%3D%20%5Cpsi%20t%20&plus;%20%5Cfrac%7Bvt%7D%7BLf%7D.%5Cdelta%20t%20.dt)

![](https://latex.codecogs.com/gif.latex?v_%7Bt&plus;1%7D%20%3D%20v%20t%20&plus;%20at%20.dt)

![](https://latex.codecogs.com/gif.latex?cte_%7Bt&plus;1%7D%20%3D%20f%28xt%29%20-%20yt%20&plus;%20%28vt.sin%28e%20%5Cpsi%20t%29%20.dt)

![](https://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt&plus;1%7D%20%3D%20%5Cpsi%20t%20-%20%5Cpsi%20des_%7Bt%7D%20&plus;%20%28%5Cfrac%7Bvt%7D%7BLf%7D%5Cdelta%20t.dt%29)

### 2.2 Timestep Length and Elapsed Duration 
In MPC, the prediction horizon is the duration (T) over which future predictions are made. The timestep length N and elapsed duration dt are used to calculate T = N * dt. 
For self driving cars, the prediction horizon changes all the time and a new set of predictions are computed again for the new environment, and the prediction horizon does not need to be very large. The values of N and dt are hyperparameters used to tune the MPC for a particular use case. Some general guidelines for choosing N and dt are: 
  * T should be as large as possible.
  * dt should be as small as possible. 
 
In this project; the final hyperparameters are N = 10, and dt = 0.1. With these hyperparameter the MPC predicts a new horizon T = (10 * 0.1) every 1 second. I did experiment with higher values for N and dt (15, 0.06) and (20,0.05) but this made the controller unstable. The suggested max_cpu_time for the ipopt solver was 0.5 but I set it 0.1 decrease the CPU effort. To help constrain the MPC, I limited the steering angle to +/- 25 degrees (0.436332 radians), and I contrained the max speed to 60 MPH. I did try higher speeds of 65MPH but the car brushed the edges a little too closely in some cases so set the upper limit to 60MPH.


### 2.3 Model Predictive Control with Latency
In a real self-driving car the reading of an actuator output is not instaneous. There will always be a certain reading latency in the sensor itself and the packet transfer latency from the sensor to the ADAS compute module. To model the latency the main thread sleeps for 100ms before sending the updates to the simulator. 

---
	
## 3. Build Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

## 4. Build Instructions 
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

