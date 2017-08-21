Self-Driving Car Nanodegree - Path Planning Project
====================================================================

In this document we describe the implementation of the Path Planning project for the course,
following the questions from the [rubric](https://review.udacity.com/#!/projects/318/rubric).

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
    
-> screenshot

2. The car drives according to the speed limit.
-----------------------------------------------

-> discussion Frenet/XY
-> Tuning per-lane speed

3. Max Acceleration and Jerk are not exceeded.
----------------------------------------------

-> Ensuring max acceleration in equations.
-> Generating jerk-min trajectories with s_dot6 = 0

4. Car does not have collisions.
--------------------------------

-> Target tracking
-> Gap tracking

5. The car stays in its lane, except or the time between changing lanes.
------------------------------------------------------------------------

-> Target 'd' is always the center of any lane

6. The car is able to change lanes.
-----------------------------------

-> Behavioral planner outputs lane changing actions
-> Attach screenshot

Reflection
==========
1. There is a reflection on how to generate paths.
--------------------------------------------------
The reflection has been included in the document [Model Documentation.md](Model Documentation.md).
