Self-Driving Car Nanodegree - Path Planning Project
====================================================================

In this document we describe the implementation of the Path Planning project for the course,
following the questions from the [rubric](https://review.udacity.com/#!/projects/318/rubric).

Demonstration video
===================

Watch [here](path_planning_final.mkv).
![](res/second_lap.png)

Dockerfile
==========
Before describing the implementation, it's worth noting the following scripts:

- `build.sh`. It will build the project using a Docker image containing
the required dependencies. Please run it from command line as:

      $ ./build.sh

- `run.sh`. It will run the project from `build/path_planning`.
Please run it from command line as:

      $ ./run.sh

This should allow reviewers to be able to build and run the project
even if they don't have the dependencies installed.
Otherwise, standard `cmake/make` build steps are possible given that
the required dependencies are available.

Rubric questions
================
Now we proceed to answer the questions from the rubric.

Compilation
===========

1. The code compiles correctly
------------------------------
The code compiles without any errors or warnings on:

- Ubuntu 16.04
- G++ 5.4.0
- CMake 3.5.1

By running the following:

    mkdir build
    cd build
    cmake ..
    make

To run the code, please follow:

    cd build
    ./path_planning

Valid Trajectories
==================

1. The car is able to drive at least 4.32 miles without incident.
----------------------------------------------------------------

This can be observed in the video and in the following picture:

![](res/second_lap.png)

Indeed, the vehicle has traveled 5.18 miles, and one lap is 4.32 miles.

In order to accomplish this, obviously a good trajectory generator is required
to avoid other cars and stay in lane (explained in detail in the
[model documentation](Model Documentation.md)).

However, two additional aspects are key to accomplish the task of
going from Lap 1 to Lap 2:

- Wrapping the `s` Frenet coordinate around the maximum `s`. This is performed
in `src/trajectory_generator.cpp:231`:

    const double s = std::fmod(evaluatePolynomial(coeffs_s, t), kMaxS);

note that we take the modulus of the target `s` with respect `kMaxS`.

- Computing differences in `s` coordinates must be taken care of
in a special way, just the way we do we angles. This is implemented
in `include/map.h`:

```cpp
static double s_min_diff(double s_a, double s_b)
{
    double output = s_a - s_b;

    if (output < -0.5 * kMaxS)
    {
        output += kMaxS;
    }
    else if (output > 0.5 * kMaxS)
    {
        output -= kMaxS;
    }

    return output;
}
```

This function ensures that the difference between `s_a` and `s_b` will
always be the shortest distance, an can be signed.

2. The car drives according to the speed limit.
-----------------------------------------------

Given that we managed to complete one lap without incidents, we prove that
the vehicle drives according to the speed limit.

Please refer to XXXX for more details.

3. Max Acceleration and Jerk are not exceeded.
----------------------------------------------

As before, we drive one lap without any issues, meaning that we also fulfill the
acceleration and jerk requirements.

Please refer to XXXX for more details.


4. Car does not have collisions.
--------------------------------

Collision avoidance is also ensured during the test lap, as shown in the picture
above.

Please refer to XXXX for more details.


5. The car stays in its lane, except or the time between changing lanes.
------------------------------------------------------------------------

The simulator also checks that the car stays in lane, so we have accomplished
that as well.

Please refer to XXXX for more details.

6. The car is able to change lanes.
-----------------------------------

This has been successfully accomplished as shown in the following pictures:

![](res/lane_change_1.png)
![](res/lane_change_2.png)

Please refer to XXX for more details.

Reflection
==========
1. There is a reflection on how to generate paths.
--------------------------------------------------
The reflection has been included in the document [Model Documentation.md](Model Documentation.md).
