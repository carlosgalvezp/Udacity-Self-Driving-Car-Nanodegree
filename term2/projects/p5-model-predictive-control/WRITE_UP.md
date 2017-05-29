Self-Driving Car Nanodegree - Model Predictive Control (MPC) Project
====================================================================

In this document we describe the implementation of the MPC project for the course,
following the questions from the [rubric](https://review.udacity.com/#!/rubrics/896/view).

Dockerfile
==========
Before describing the implementation, it's worth noting the following scripts:

-`build.sh`. It will build the project using a Docker image based on the
present `Dockerfile`. Please run it from command line as:

    $ ./build.sh

-`run.sh`. It will run the project from `build/mpc`. Please run it from command
line as:

    $ ./run.sh

This should allow reviewers to be able to build and run the project
even if they don't have the dependencies installed.

Rubric questions
================
Now we proceed to answer the questions from the rubric.

1.  The Model
-------------

The MPC approach consists on predicting the future trajectory of the vehicle
over a time horizon, given its initial state. A cost function over the horizon
is computed, and then passed to a non-linear optimizer that will find
the best overall trajectory fulfilling a number of constraints.

The model consists of the following:

-**Trajectory**

The model starts with the desired trajectory that the vehicle
should follow. It is represented as a third-order polynomial, in
**local vehicle coordinates**:

    f(x) = ax^3 + bx^2 + cx + d

-**State vector**

The state vector of the vehicle is a 6-dimensional vector:

    [x, y, psi, v, cte, epsi]

where:

  -`x` is the X position of the vehicle                       [m]

  -`y` is the Y position of the vehicle                       [m]

  -`psi` is the orientation of the vehicle                    [rad]

  -`v` is the velocity of the vehicle                         [m/s]

  -`cte` is the cross-track error = f(x) - y                  [m]

  -`epsi` is the orientation error = psi - atan(f'(x))        [rad]

We choose to input this vector in **local coordinates**, so the initial
state always has `x = y = psi = 0`.

-**Actuators**

The actuators of the vehicle are a 2-dimensional vector:

    [delta, acc]

where:

  -`delta` is the steering commands [rad]. Normalized to [-1, 1] before passing
   to the simulator.

  -`acc` is the accelerator/throttle commands.
   A positive value indicates acceleration, whereas a negative value means braking.

-**Update equations**

Finally, the model describes how the state of the vehicle
at time `t` evolves towards the state at the next time step,
`t+1 = t + dt`, where `dt` is the timestep duration.
For this project we use the following kinematic vehicle model:

    x_t+1    = x_t    + v_t * cos(psi_t) * dt
    y_t+1    = y_t    + v_t * sin(psi_t) * dt
    psi_t+1  = psi_t  + (v_t/Lf) * delta_t * dt
    v_t+1    = v_t    + acc_t * dt
    cte_t+1  = cte_t  + v_t * sin(epsi_t) * dt           [*]
    epsi_t+1 = epsi_t + (v_t/Lf) * delta_t * dt          [**]

where `Lf` is the length between the center of gravity and the front of the car.

[*] For a more accurate representation, we can substitute:

    cte_t = f(x_t) - y_t

[**] For a more accurate representation, we can substitute:

    epsi_t = psi_t - psi_des_t = psi_t - atan(f'(x_t))

-**MPC Horizon**

The horizon in an MPC problem is described as the time
in the future where we predict and optimize the vehicle trajectory.
It is defined by:

  -`N`. Number of steps. We choose `N = 10` (see motivation later).

  -`dt`. Timestep duration. We choose `dt = 0.05 seconds` (see motivation later).

  -`T = N * dt = 1 seconds`. Time horizon.

-**Optimization problem**

In order to optimize for the overall vehicle
trajectory, we create a vector with 6N + 2(N-1) components, since
we have 6N state variables and 2(N-1) actuator commands.
The vector is packed as follows:

    [x0 x1 ... xN-1, y0 y1 ... yN-1, ... ,epsi0 ... epsiN-1, delta0 ... deltaN-2, acc0 ... accN-2]

-**Cost function**

The model will optimize a cost function based on the complete state vector above.
The cost function is defined in Lines 176-200 of `src/optimizer.cpp`.

First, we ensure the vehicle stays in the center of the road, well orientated
and keeping the target velocity (40 mph), adding to the cost function:

  -The sum for t = 0 ... N of `cte_t^2`.

  -The sum for t = 0 ... N of `epsi_t^2`.

  -The sum for t = 0 ... N of `(v_t - ref_v)^2`, where `ref_v` = 40 mph.

```
// Minimize CTE and epsi, and keep target velocity
for (std::size_t i = 0U; i < kHorizonSteps; ++i)
{
    cost += CppAD::pow(x[kIdxCTE_start  + i] - kRefCte,  2);
    cost += CppAD::pow(x[kIdxEpsi_start + i] - kRefEpsi, 2);
    cost += CppAD::pow(x[kIdxV_start    + i] - kRefV,    2);
}
```

Next, we enforce a minimal use of actuators - thus having lower
energy consumption, adding:

  -The sum for t = 0 ... N-1 of `delta_t^2`

  -The sum for t = 0 ... N-1 of `acc_t^2`

```
// Minimize the use of actuators
for (std::size_t i = 0U; i < kHorizonSteps - 1U; ++i)
{
    cost += CppAD::pow(x[kIdxDelta_start + i], 2);
    cost += CppAD::pow(x[kIdxAcc_start   + i], 2);
}
```

Finally, we want smooth actuator commands. We had to add a weight of 500 on the
steering command to avoid hard oscillations in the vehicle, keeping
smooth turns and trajectories. We add the following terms to the cost function:

  -The sum for t = 0 ... N-2 of `500*(delta_t+1 - delta_t)^2`

  -The sum for t = 0 ... N-2 of `(acc_t+1 - acc_t)^2`

```
// Minimize the value gap between sequential actuations.
for (std::size_t i = 0U; i < kHorizonSteps - 2U; ++i)
{
    cost += 500.0 * CppAD::pow(x[kIdxDelta_start + i + 1] - x[kIdxDelta_start + i], 2);
    cost +=         CppAD::pow(x[kIdxAcc_start   + i + 1] - x[kIdxAcc_start   + i], 2);
}
```

-**Constraints**

The following constraints were posed in the optimization problem:

  -The initial state cannot change (it's the one given by the simulator).
   This is implemented in Lines 202-209 of `src/optimizer.cpp`:

```
// g_0 = x, so to make sure the initial state does not change during optimization
fg[1 + kIdxPx_start]   = x[kIdxPx_start];
fg[1 + kIdxPy_start]   = x[kIdxPy_start];
fg[1 + kIdxPsi_start]  = x[kIdxPsi_start];
fg[1 + kIdxV_start]    = x[kIdxV_start];
fg[1 + kIdxCTE_start]  = x[kIdxCTE_start];
fg[1 + kIdxEpsi_start] = x[kIdxEpsi_start];
```

   And in lines 134-167 of `src/optimizer.cpp`:


```
void Optimizer::updateInitialState(const Eigen::VectorXd& state)
{
    // Retrieve data
    const double px   = state[0U];
    const double py   = state[1U];
    const double psi  = state[2U];
    const double v    = state[3U];
    const double cte  = state[4U];
    const double epsi = state[5U];

    // Setup the initial state at t = 0
    variables_[kIdxPx_start]   = px;
    variables_[kIdxPy_start]   = py;
    variables_[kIdxPsi_start]  = psi;
    variables_[kIdxV_start]    = v;
    variables_[kIdxCTE_start]  = cte;
    variables_[kIdxEpsi_start] = epsi;

    // Setup the upper and lower bounds of the constraints for the state
    // at t = 0 i.e. it cannot be modififed
    constraints_lowerbound_[kIdxPx_start]   = px;
    constraints_lowerbound_[kIdxPy_start]   = py;
    constraints_lowerbound_[kIdxPsi_start]  = psi;
    constraints_lowerbound_[kIdxV_start]    = v;
    constraints_lowerbound_[kIdxCTE_start]  = cte;
    constraints_lowerbound_[kIdxEpsi_start] = epsi;

    constraints_upperbound_[kIdxPx_start]   = px;
    constraints_upperbound_[kIdxPy_start]   = py;
    constraints_upperbound_[kIdxPsi_start]  = psi;
    constraints_upperbound_[kIdxV_start]    = v;
    constraints_upperbound_[kIdxCTE_start]  = cte;
    constraints_upperbound_[kIdxEpsi_start] = epsi;
}
```

  -The states at t > 0 must change according to the vehicle motion model
  described above, see Lines 211-261 of `src/optimizer.cpp`:

```
// Vehicle model
const double dt = kDeltaT;

fg[2 + kIdxPx_start + t]   = x_t1    - (x_t + v_t * CppAD::cos(psi_t) * dt);
fg[2 + kIdxPy_start + t]   = y_t1    - (y_t + v_t * CppAD::sin(psi_t) * dt);
fg[2 + kIdxPsi_start + t]  = psi_t1  - (psi_t + (v_t/Lf) * delta_t * dt);
fg[2 + kIdxV_start + t]    = v_t1    - (v_t + acc_t * dt);
fg[2 + kIdxCTE_start + t]  = cte_t1  - (cte_t + v_t * CppAD::sin(epsi_t) * dt);
fg[2 + kIdxEpsi_start + t] = epsi_t1 - (epsi_t + (v_t/Lf) * delta_t * dt);
```

-**Variables boundaries**

Finally, we must also set boundaries for the variables involved in the
complete vector passed to the optimizer:

  -State variables: can vary between `std::numeric_limits<double>::lowest()` and `max()`

  -Steering actuator: can vary between `-degtoRad(25.0)` and `degToRad(25º)` [rad]

  -Accelerator actuator: can vary between -1.0 and 1.0 [m/s^2]

This is implemented in Lines 99-121 of `src/optimizer.cpp`:

```
void Optimizer::initializeVariablesBounds()
{
    // State variables can have any value
    for (std::size_t i = 0U; i < kIdxDelta_start; ++i)
    {
        variables_lowerbound_[i] = std::numeric_limits<double>::lowest();
        variables_upperbound_[i] = std::numeric_limits<double>::max();
    }

    // Steering limitations [rad]
    for (std::size_t i = kIdxDelta_start; i < kIdxDelta_end; ++i)
    {
        variables_lowerbound_[i] = -kMaxSteering;
        variables_upperbound_[i] =  kMaxSteering;
    }

    // Acceleration limitations [m/s^2]
    for (std::size_t i = kIdxAcc_start; i < kIdxAcc_end; ++i)
    {
        variables_lowerbound_[i] = -kMaxAcc;
        variables_upperbound_[i] =  kMaxAcc;
    }
}
```

**2. Timestep length and elapsed duration (N and dt)**
------------------------------------------------------
Another important factor to tune in an MPC is the horizon, defined by the
number of prediction steps in the future (`N`) and the timestep duration (`dt`).

We have chosen the following values:

    N = 10
    dt = 0.05

This gives us a prediction horizon of `T = 0.5 seconds`, which we found was enough
for a target speed of `v_ref = 40 mph`.

Now we describe other values tried, in order to motivate the final choice.

-**Tuning N**

First, we tried larger values of `N`, starting at `N = 25` and going
down to 10 in steps of 5. We noticed that the larger the value of `N`, the larger
the **computation time**. This makes sense because the size of
the state vector passed to the optimizer is proportional to `N`. Therefore
the bigger the state the more variables to optimize. Even though the trajectory
could be very good (one can see very far ahead), there is a huge latency
involved due to the computation time, which renders the trajectory useless.
Therefore we lowered `N` down to 10 to have a computation time between 10-20 ms.
We didn't see significant performance drop by doing this, but the computation
time went down pretty much.

Just for fun, we also tried `N = 5`. In this case the horizon was too short
and actually the vehicle ended up either accelerating and braking (not moving)
or turning very sharply going off road very quickly. In sum, a too short
horizon is not good since you cannot see a reasonably far section of the
overall trajectory.

-**Tuning dt**

Once we were satisfied with the computation time from `N`, we tune `dt` to
determine how far into the future (in time) we can predict.

We started with `dt = 0.1` to obtain `T = 1 second` of horizon. The vehicle
performance was very good in the beginning at low speeds. However when
approaching 40 mph the horizon extended too far into the future (the green line
was very long), which made the optimization much harder since the trajectory
had curves far in the future that maybe the vehicle didn't even have to consider
at that point. Because of this, the computation time would raise sometimes
to quite large values, and as a consequence the vehicle would start to oscillate
due to the latency.

Then we moved down to `dt = 0.05` which we found to be a very good time step. It
also provides a better approximation of the reference trajectory since 
the time difference between time steps is smaller.
At 40 mph, the horizon (green line) looked shorter than before
in the simulator, which made the vehicle focus on optimizing only the necessary
trajectory ahead to keep straight or take curves, excelling in both cases.

For fun we also tried `dt = 0.01`, but as expected we have the same problems
as with small `N`: the horizon is too small and the vehicle can't see much of
the trajectory ahead, leading to sharp movements and oscillations, eventually
goinf off the road.


**3. Polynomial Fitting and MPC Preprocessing**
-----------------------------------------------
The following preprocessing is applied before calling the MPC routine:

  -Transform velocity from mph to m/s [S.I.] - Line 77 in `src/main.cpp`:

```
v = Tools::mphtoms(v);                       // [m/s]
```

  -Transform waypoints from global to local vehicle coordinate system.
  Lines 79-89 in `src/main.cpp`:

```
// Convert trajectory points from global to local frame
Eigen::VectorXd xvals(ptsx.size());
Eigen::VectorXd yvals(ptsy.size());
const double c = std::cos(psi);
const double s = std::sin(psi);

for (std::size_t i = 0U; i < ptsx.size(); ++i)
{
    xvals[i] =  c * (ptsx[i] - px) + s * (ptsy[i] - py);
    yvals[i] = -s * (ptsx[i] - px) + c * (ptsy[i] - py);
}
```

  where `xvals` and `yvals` are `Eigen::VectorXd` that we can pass to the `polyfit()` function.

  -Transform state vector into local vehicle coordinates - Lines 95-98 of `src/main.cpp`:

```
// Setup vehicle state in local coordinates
const double x_local = 0.0;
const double y_local = 0.0;
const double psi_local = 0.0;
```
   NOTE: `v` is unchanged.

  -Compute CTE and epsi to pass a 6-dimensional state vector to the MPC,
   as shown in Lines 100-107. The computations are simplified since we use
   local coordinates:

```
// Compute CTE and epsi in local coordinates
const double cte_local = Tools::polyeval(trajectory, x_local);

// epsi = psi - atan(f'(x)) where f is the trajectory
// Since we evaluate at x_local = 0, the only remaining
// term is trajectory[1]
const double psi_traj = std::atan(trajectory[1]);
const double epsi_local = psi_local - psi_traj;
```

  -Create the 6-dimensional state vector that we pass to the MPC routine
   in Lines 106-107 of `src/main.cpp`:

```
// Create state vector in local coordinates
Eigen::VectorXd state(6);
state << x_local, y_local, psi_local, v, cte_local, epsi_local;
```

  -After the call to MPC, we perform the following **post-processing** of
  the steering command:

    1. Normalize to the range [-1, 1] by dividing by `degToRad(25)`.
    2. Switch sign to match the simulator angle sign convention for rotations.

  This is done in Lines 130 and 137 of `src/main.cpp`:
```
const double steering = commands.steering / Tools::deg2rad(25.0);
msgJson["steering_angle"] = -steering;

```

**4. Model Predictive Control with Latency**
--------------------------------------------
The way we handle latency in this project is by modifying the motion model,
in `src/optimizer.cpp`.
The effect of latency in the system is that the vehicle keeps moving
with the same actuators during the latency period (in this case, 100 ms),
and then new actuators are applied after this time.

Therefore, it makes sense to predict an optimal trajectory starting from a
**predicted state of the vehicle after 100 ms**. The additional prediction
of the current state 100 ms into the future is implemented
in Lines 232-238 of `src/optimizer.cpp`:

```
// Predict current state 100 ms into the future to account for latency
// Assumming constant the previous actuator commands
const double latency = 0.1;  // [s]
x_t   = x_t   + v_t * CppAD::cos(psi_t) * latency;
y_t   = y_t   + v_t * CppAD::sin(psi_t) * latency;
psi_t = psi_t + (v_t/Lf) * delta_t * latency;
v_t   = v_t   + acc_t * latency;
```

As can be seen, we do **not** use the normal variables at time `t`,
such as `x_t`, `y_t` and so on that are defined a few lines above,
which are part of the complete state vector.

Instead, we take those varibles and predict the real ones after the latency of 100 ms.
Only after we perform this operation we apply the complete vehicle model.

The resulting effect is that, after every step in the horizon, the MPC
tries to optimize for a trajectory in which the commands don't arrive
to the vehicle until 100 ms after they have been sent.

This simple strategy completely removes the effect of latency - great
oscillations in the vehicle to the point that it went out of the road.
With this modification in the code, the vehicle can complete a lap
without any oscillation or problems, just as if there was no latency
at all!

**NOTE**: it is NOT enough to perform this latency prediction only
for `t = 0`, i.e. before sending the state to the MPC, in `src/main.cpp`.
Otherwise, the MPC will optimize for a trajectory where all but the first state
are delayed, which is an incorrect assumption - there is latency
at **every** time step, and therefore it needs to be accounted for
inside the `for loop` over the `N` steps of the horizon.
