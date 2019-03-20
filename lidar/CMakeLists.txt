# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(lidar)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lidar src/deflection.cpp src/SensorData.cpp src/main.cpp)
target_link_libraries(lidar ${catkin_LIBRARIES})
add_dependencies(lidar beginner_tutorials_generate_messages_cpp)

# %EndTag(FULLTEXT)%
