# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
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
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Model
The motion model for the car is based on CRTV (Constant Rate of Turn and Velocity) between the actuation steps assuming actuations are atomic. The state is defined as a six-dimension vector as follows:
```
	x - position on the x axis
	y - position on the y axis
	psi - orientation
	v - speed of the vehicle
	cte - cross track error
	epsi - orientation error
```
All values are based on vehicle coordinate system (current position and orientation are 0)
Under this model the transition of the state between the steps is defined as follows:
```
	x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
	y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
	psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
	v_[t+1] = v[t] + a[t] * dt
	cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
	epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
``` 

## Planning Horizon
The number of steps and duration of each step were chosen empirically as follows:
* N  = 9   - longest visible horizon while driving at 60mph
* dt = 0.2 - shortest time between actuations (including all latency sources)

Lower values of dt cause oscillations because the actuation is performed to late after accounting for the latency, higher values reduce the controllability of the vehicle and required reducing speed.
As for the N - with smaller values the planning horizon is too close and might result in more aggressive actuations which cause oscillations. Higher values are not useful because they exceed the curvature represented by the waypoints.

## Coordinate System
The waypoints are received from the simulator in absolute coordinate system (map coordinates) but the state is expressed in vehicle coordinate system. Therefore, the code performs conversion from map to car coordinate system as follows:
```
    double dx = ptsx[i] - px;
    double dy = ptsy[i] - py;
    ptsx[i] = dx*cos(psi) + dy*sin(psi);
    ptsy[i] = -(dx*sin(psi) - dy*cos(psi));
```

## Latency
There are three main latency sources in the system:
* Synthetic - a sleep command of 100ms in the MPC code to simulate fixed latency
* Computational - the time it takes for optimizer to find the optimal actuation values, can vary depending on the machine but usually < 100ms
* Network - when running the simulator and the controller code on different machines connected by network (such as running the simulator on the laptop and controller on AWS instance), network delay is introduced that can be 20ms-100ms depending on the location. This latency is not fixed like the synthetic one and can vary between the frames

Unfortunately the network latency presents a big problem due to the latency jitter therefore even if the code uses some fixed actuation step in the future and assumes that control values are held constant before that step it will not work when the latency changes. The only way to deal with this problem is to increase dt which guarantees that the time between steps is larger than the time between possible actuations. The downside of it is reduced controllability, meaning that on sharp turns the speed must be reduced to achieve controllability

## Dynamic Adaptability
Due to (mostly) latency constraints the controller is unable to handle sharp turns with speeds exceeding 35mph, however on more straight sections of the track the speed can be increased. To achieve that several constraints and dependencies were introduced to the code as follows:
* Speed - the reference speed depends on the curvature of the polynomial representing the waypoints and the current CTE `double ref_v = ref_v = 25 + 0.4*curv_factor - fabs(coeffs[0])`
* Sequential actuations gap - the weight of the gap between sequential actuations in cost function depends on the curvature of the road - the higher the curvature the lower is the minimization weight.
* Drag - to model deceleration created by friction a constant deceleration factor of 0.05 is subtracted from every actuation value
* Turn rate - is dynamically constrained by inversely dependent on the speed - the higher the speed the lower are the steering values (in absolute terms)
* Location - the controller assumes that the vehicle moves forward, therefore in car coordinate system the location value on the x axis must monotonically increase

## Results
Below is a video showing simulator running at an average 60mph on a laptop, sending telemetry over the network to AWS server running the controller (this code) and receiving back the actuation commands and planned path. On top of that synthetic latency of 100ms for each from is introduced in the code as per the project requirements.
[![Watch the video](https://j.gifs.com/DRvyK5.gif)](https://youtu.be/wXaEUJdqAa8)
