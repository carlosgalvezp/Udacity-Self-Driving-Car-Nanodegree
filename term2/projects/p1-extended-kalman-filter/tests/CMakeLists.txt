include_directories(3rdparty/googletest-release-1.8.0/googletest/include)
include_directories(3rdparty/googletest-release-1.8.0/googlemock/include)

function(build_test test_name)
    list(REMOVE_AT ARGV 0)
    add_executable(${test_name} ${ARGV})
    target_link_libraries(${test_name} gtest gmock gmock_main ekf_lib)
endfunction(build_test)

build_test(test_tools tools_test.cpp)

