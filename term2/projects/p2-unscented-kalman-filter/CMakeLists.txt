project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

option(build_tests "Build the tests." OFF)

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

#if(build_tests)
#    add_subdirectory(3rdparty/googletest-release-1.8.0)
#    add_subdirectory(tests)
#endif()
