# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Model
This implementation of model predictive control uses a state vector consisting of:
* absolute 2D position [x,y]
* velocity [v]
* steering angle [psi]
* cross track error [cte]
* trajectory error [epsi]

Error values are computed at each timestep based on the reference path that is provided to the model.

The actuators (control variables) available to the model for control are:
* Throttle
* Steering angle

Using the reference path points, a fit curve is created representing the desired trajectory over time.  The solver then takes the state, vehicle model (equations governing how varying control variables affect trajectory), specification of actuator constraints, a cost function to discourage solutions with undesirable "real world behavior", as well as a target velocity which (essentially_ gives the model a "reason" for forward progress, as cross-track error and angle error alone do not) .  The output (solution) of this process are actuation values producing a fit curve that will (hopefully!) lead to the the vehicle following the reference trajectory at the maximum manageable speed. 

### Timestep Length and Elapsed Duration

Two critical variables for the model are N and dt - the number of control steps and time between steps, respectively.  This determines the number of points the solver has to work with per timestep, as well as the total duration of control planning.  For exmaple, N=10 at dt=100ms would mean the model is planning control for 1 second in the future (100ms * 10).  The final variables chosen in this model were N=6 and dt=0.1 (100ms).  Other values tried were in the range N=[3,15] and dt=[.01,.2].  Both of these resulted in undesireable performance with the following general observations:

* N > 6 caused steering osciallation - meaning the vehicle tended towards following a sine-wave like trajectory.  This ranged from small, continuous oscillations that could be considered a minor problem to a feedback-loop where oscillations steadily increased in magnitude until loss of control/crash.
* N < 6 resulted in an inability to follow the reference trajectory whatsoever (more on this below)
* dt < .1 results in steering oscillation with feedback loop (increasing magnitude over time) and eventual loss of control
* dt > .1 results in planning too far into the future, causing the car to literally cut corners

I was unsuccessful in determing why such small changes in these values, specifically N, produce a seemingly unreasonable effect on the model's performance.  For example setting N=5, the vehicle crashes almost immediately in the simulator.  At N=7, large osciallations are created during cornering, and the vehicle crashes in the S-bend section of the track.  However, at N=6, the model is performing nearly flawlessly with neither of these issues...  I would like to understand this better - intuitively it does not seem that the model should be so unstable with respect to +/- 1 N, or +/- 100ms to the planning period.  My thought so far is that this may be related to the artifical 100ms actuation delay in the simulator (especially oscillatory steering). 

### Pre-processing of data points

In order to simplify the math for the model calculations, as well as for displaying trajectories in the simulator, points from the simulator are converted to vehicle-relative coordinates prior to processing, i.e. placing the vehicle at 0, 0.  This means the initial/current state vector passed to the MPC solver always has x=1,y=0,psi=0.  I used x=1 as a means for attempting to compensate for actuator delay, so the initial position is estimated slightly in the future.  This had a mild but positive effect on performance in corners and at higher rates of speed.  I am not sure psi should necessarily be 0, but was unable to get the model working correctly with any transformed version of that value.

### Cost functions

One sigificant increase in my model's performance came from modification of the cost functions.  In the end I chose an approach which variably penalizes errors based on whether they are near in time or father away.  The costs at each step have a coefficient in the form of (C / t) or (t / C) where C is a constant and t is the timestep in the current prediction.  As such, the weight of the cost at each step in a given control solution will either become linearly larger or smaller with respect to how near or far away in time it is.  E.g. for cross-track error, error at the very next timestep is given a 1/6th the weight of error at the 6th timestep.  This is because it is less important the vehicle harshly "jerk" into the optimal trajectory within 100ms than it is to ensure the vehicle is on a path to being on the optimal trajectory within the next 600ms.  

It took some experimentation to find good coefficients for the cost functions, and care had to be taken to roughly normalize the magnitudes of variables. e.g. the cost for epsi used a magnitude coefficient of 40, while cte has 15.  This is partly due to weighting the cost based on type of error, but also due to the fact that the real possible magnitude range of these values are not identical.

### Achieved Performance

The model is able to successfully manuver the track with a target speed of 80mph, within project constraints.  The model is actually capable of manuvering the track with a 100mph target, which is by far the fastest I've been able to accomplish with any control model, but unfortunately touches the curb twice which violates acceptance criteria.

### Open Problems and Topics for Further Investigation

* There seems to be some problem with displaying the MPC-output predicted trajectory in the simulator.  I am not sure if this is a visual artifact, or if there is some problem with the actual prediction.  Sometimes it is very accurate, but at several times during the simulation the last one or two points form a harsh "dog leg" that doesn't seem correct.
* Per the discussion above in "Timestep Length and Elapsed Duration", it would be educational to determine why small changes in N and dt seem to have an unreasonable effect on model performance.  I believe inbex calculations etc. are all correctly calculated dynamically, but there may be a coding error somewhere that unintentally depends on N=6
* I believe the model can be improved further.  In its current form, I was unable to improve one dimention of performance without sacrificing another, e.g. improve sharp-cornering behavior while maintaining smooth control in straights and gradual turns.  Along with the N/dt topic, a better understanding of the underlying math in the MPC solver is likely necessary to improve. 


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
