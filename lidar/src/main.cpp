#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <deflection.hpp>

ros::Publisher* publisher = nullptr;

bool messages_written = false;

void on_lidar_message(const sensor_msgs::LaserScan::ConstPtr& message) {
    double deflection = calculate_deflection(message);

    std_msgs::Float64 msg;
    msg.data = deflection; 
    publisher->publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar");
    ros::NodeHandle n;

    // set up the publisher
    ros::Publisher pub = n.advertise<std_msgs::Float64>("theta_deflection", 1);
    publisher = &pub;

    // set up the subscriber
    ros::Subscriber sub = n.subscribe("scan", 1, on_lidar_message);

    // run endlessly
    ROS_INFO("Initialized lidar reactive control system");
    ros::spin();
    return 0;
}

