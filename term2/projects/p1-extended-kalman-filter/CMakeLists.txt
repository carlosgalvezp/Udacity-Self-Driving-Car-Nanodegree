project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x -Wall -Wextra -Werror)

include_directories(include)
include_directories(3rdparty)

set(sources
    src/FusionEKF.cpp
    src/kalman_filter.cpp
    src/tools.cpp)

add_library(ekf_lib ${sources})

add_executable(ExtendedKF src/main.cpp)
target_link_libraries(ExtendedKF ekf_lib)

add_subdirectory(3rdparty/googletest-release-1.8.0)
add_subdirectory(tests)
