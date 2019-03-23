# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(lidar)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#SET(SOURCES src/math.cpp src/deflection.cpp src/main.cpp)
SET(SOURCES src/main.cpp)
add_executable(lidar ${SOURCES})
target_link_libraries(lidar ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%
