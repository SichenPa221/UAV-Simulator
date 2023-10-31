cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(UAVController)

# Add the executable
add_executable(UAVController FlyRobotSim.cpp)

# Set C++ standard
set_target_properties(UAVController PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

