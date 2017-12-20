# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Submission
This repository contains my code which I have submitted as part of the nanodegree term 2. In the following four sections I will answer questions relating my implementation.
For build instructions, see *Basic Build Instructions* below.


## The Effect of Each of the P, I, D Components in the implementation
For this demonstration, we consider only the steering controller.

### Proportional controller
To demonstrate the effect of solely the proportional controller, set `Kd=Ki=0`.

[Link to video demonstrating proportional controller only (ogv-format)](./image/videos/Kp_only.ogv)

The proportional controller reacts (anti-)proportional to the cross tracking error, i.e. the steering angle is the greatest when the vehicle is the furthest away from the middle line. However, when the vehicle then crosses the middle line, the steering angle is equal to zero. This results, unfortunately, in a tendency to overshoot.


### Differential Controller
To demonstrate the effect of solely the proportional controller, set `Kp=Ki=0`.

[Link to video demonstrating differential controller only (ogv-format)](./image/videos/Kd_only.ogv)

The differential controller keeps the axis of the vehicle aligned with the lane.
Using the differential controller alone, however, has the disadvantage that the vehicle has no incentive to return to the middle of the lane.

### Integration Controller
To demonstrate the effect of solely the proportional controller, set `Kp=Kd=0`.

[Link to video demonstrating integral controller only (ogv-format)](./image/videos/Ki_only.ogv)

The integration controller provides a memory, i.e. the longer the vehicle stays on one side of the middle line, the stronger is the tendency of the controller to counteract.

### PID Controller
Superposition of all three types of the above controllers yields the PID controller which keeps the vehicle on the middle line.

[Link to video demonstrating all three controllers working together (ogv-format)](./image/videos/demo.ogv)


## Choice of the Final Hyperparameters
One lessons I have learned in physics is that we first have to obtain an idea of the orders of magnitude we are dealing with. Therefore I've started with manually changing the parameters by setting one of them to zero (`Kp` in this case), and multiply or divide the others by orders of ten in relation to `Kp`.


After finding a suitable combination, I was able to tweak the magnitude of all three paramters by merely changing the value of `Kp`.

```
Kp_steering = 5.0
Ki_steering = 0.01 * Kp
Kd_steering = 100 * Kp
```

With the throttle PID controller, I have pursued a similar approach and ended up with the simple set of parameters:

```
Kp_throttle = 1.0
Ki_throttle = 0.01 * Kp
Kd_throttle = 100 * Kp
```


## Twiddle Implementation
The class `PID` contains a method called `Twiddle`.  This method is an implementation of a simple twiddle algorithm. In the function `main`, we initialize *two* instances of PID, `pid_steering` and `pid_throttle`, in order to control both dimensions.






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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.
