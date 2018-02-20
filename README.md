# CarND-Controls-MPC
## Self-Driving Car Engineer Nanodegree Program

### Rubric Points:

#### The Model

Dynamic models aim to embody the actual vehicle dynamics as closely as possible.

They might encompass tire forces, longitudinal and lateral forces, inertia, gravity, air resistance, drag, mass, and the geometry of the vehicle.

Not all dynamic models are created equal! Some may consider more of these factors than others.

Advanced dynamic models even take internal vehicle forces into account - for example, how responsive the chassis suspension is.

I used a dynamic model. The state vector is as follow:

State = [x,y,ψ,v,cte,eψ].

I used steering angle ,“δ”,and accelerator and brake pedal ,“a”, as actuators.

The reference trajectory is typically passed to the control block as a polynomial. This polynomial is usually 3rd order, since third 
order polynomials will fit trajectories for most roads. So I used a 3rd order polynomial as reference trajectory following equations to update state vector:

x_(t+1)=x_t+v_t∗cos(ψ_t)∗dt

y_(t+1)=y_t+v_t∗sin(ψ_t)∗dt

ψ_(t+1)=ψ_t+Lf*v_t∗δ∗dt

v_(t+1)=v_t+a_t∗dt

cte_(t+1)=f(x_t)−y_t+(v_t∗sin(eψ_t)∗dt)

This can be broken up into two parts:

f(x_t)−y_t being current cross track error.

v_t∗sin(eψ_t)∗dt being the change in error caused by the vehicle's movement.

eψ_(t+1)=ψ_t−ψdes_t+(Lf*v_t∗δ_t∗dt)

Similarly to the cross track error this can be interpreted as two parts:

ψ_t−ψdes_t being current orientation error.

Lf*v_t∗δ_t∗dt being the change in error caused by the vehicle's movement.

In the classroom you've referred to the ψ update equation as:

ψ_(t+1)=ψ_t+Lf*v_t∗δ_t∗dt

Note if δ is positive we rotate counter-clockwise, or turn left. In the simulator however, a positive value implies a right turn and a negative value implies a left turn. Two possible ways to get around this are:

Change the update equation to ψ_(t+1)=ψ_t−Lf*v_t∗δ_t∗dt

Leave the update equation as is and multiply the steering value by -1 before sending it back to the server.

I used 2nd solution.

#### Timestep Length and Elapsed Duration (N & dt)

In the case of driving a car, T should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future.

The prediction horizon is the duration over which future predictions are made. We’ll refer to this as T.

T is the product of two other variables, N and dt.

N is the number of timesteps in the horizon. dt is how much time elapses between actuations. For example, if N were 20 and dt were 0.5, then T would be 10 seconds.

N, dt, and T are hyperparameters we will need to tune for each model predictive controller you build. However, there are some general guidelines. T should be as large as possible, while dt should be as small as possible.

These guidelines create tradeoffs.

The goal of Model Predictive Control is to optimize the control inputs: [δ,a]. An optimizer will tune these inputs until a low cost vector of control inputs is found. The length of this vector is determined by N:

[δ1,a1,δ2,a2,...,δN−1,aN−1]

Thus N determines the number of variables optimized by the MPC. This is also the major driver of computational cost.

MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

A good approach to setting N, dt, and T is to first determine a reasonable range for T and then tune dt and N appropriately, keeping the effect of each in mind.

I used N=10 and dt=0.1. It just works fine.

#### Polynomial Fitting and MPC Preprocessing

I got x,y from the simulator in map coordinates and transformed that to vehicle coordinate space. Then used polyfit() function to find coefficients of fitted polynomial. Then used these coefficients to evaluate car’s position and find “cte”. Because now we are in vehicle space, therefore x and y are zero. To find error of steering angle we need to calculate derivative of 3rd order polynomil in x = 0 and y = 0 and use atan() function. After that we need to calculate state vector in x = 0, y = 0 and psi = 0 (because of previous transformation) and pass this state vector to mpc.solve() function and get steerning angle and throttle value. 

#### Model Predictive Control with Latency

I used dt = 0.1 latency and calculate state vector by that befor passing state vector to mpc.solve().

#### Results

You can find a video on performance of MPC in the following link: 
https://youtu.be/kETyRTQQE_g

---

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
