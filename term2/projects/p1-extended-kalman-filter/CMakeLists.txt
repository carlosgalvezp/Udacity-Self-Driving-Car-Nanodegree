project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/FusionEKF.cpp
    src/kalman_filter.cpp
    src/main.cpp
    src/tools.cpp)

include_directories(include)
include_directories(3rdparty)

add_executable(ExtendedKF ${sources})

add_subdirectory(3rdparty/googletest-release-1.8.0)
add_subdirectory(tests)
