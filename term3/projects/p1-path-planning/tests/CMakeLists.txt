include_directories(3rdparty/googletest-release-1.8.0/googletest/include)
include_directories(3rdparty/googletest-release-1.8.0/googlemock/include)

add_executable(unit_tests utils_tests.cpp)
target_link_libraries(unit_tests gtest gmock gmock_main path_planner_lib)

