cmake_minimum_required(VERSION 2.8.3)
project(reactive_controls)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs genmsg)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE sources src/*.cpp)
file(GLOB_RECURSE include include/*.hpp) # only for IDE support
file(GLOB_RECURSE tests test/*.cpp)

add_executable(reactive_controls ${include} ${sources})
target_link_libraries(reactive_controls ${catkin_LIBRARIES})

catkin_add_gtest(unit_tests ${tests})
target_link_libraries(unit_tests ${catkin_LIBRARIES})
