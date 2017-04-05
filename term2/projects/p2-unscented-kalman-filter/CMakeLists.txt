project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x -Wall -Wextra -Werror -pedantic -O2)

include_directories(include)
include_directories(3rdparty)

set(sources
    src/fusion_ukf.cpp
    src/measurement_model.cpp
    src/measurement_model_lidar.cpp
    src/measurement_model_radar.cpp
    src/motion_model.cpp
    src/tools.cpp
    src/ukf.cpp)

add_library(ukf_lib ${sources})

add_executable(UnscentedKF src/main.cpp)
target_link_libraries(UnscentedKF ukf_lib)
