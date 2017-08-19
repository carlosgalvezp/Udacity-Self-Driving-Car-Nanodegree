project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

option(build_tests "Build the tests." OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wextra -Werror -pedantic")

include_directories(include
                    3rdparty/Eigen-3.3
                    3rdparty/spline)

set(sources src/behavior_planner.cpp
            src/map.cpp
            src/path_planner.cpp
            src/trajectory_generator.cpp
            src/utils.cpp
            src/vehicle.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_library(spline_lib 3rdparty/spline/spline.cpp)

add_library(path_planner_lib ${sources})
target_link_libraries(path_planner_lib spline_lib)

add_executable(path_planning src/main.cpp)
target_link_libraries(path_planning path_planner_lib z ssl uv uWS)

if(build_tests)
    add_subdirectory(3rdparty/googletest-release-1.8.0)
    add_subdirectory(tests)
endif()
