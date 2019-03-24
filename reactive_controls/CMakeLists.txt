# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(reactive_controls)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs genmsg)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#SET(SOURCES src/math.cpp src/deflection.cpp src/main.cpp)
SET(SOURCES src/main.cpp)
add_executable(reactive_controls ${SOURCES})
target_link_libraries(reactive_controls ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%
