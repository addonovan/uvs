#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <deflection.hpp>
#include <math.hpp>

ros::Publisher* publisher = nullptr;

void on_lidar_message(const sensor_msgs::LaserScan::ConstPtr& message) {
    double deflection = calculate_deflection(message);

    // for large angles we should report the changes we're making
    if (deflection >= 1.0 || deflection <= -1.0) {
        ROS_INFO("Deflecting by %f°", deflection);
    }

    // respond with a 64-bit float telling by how many radians we should deflect
    std_msgs::Float64 msg;
    msg.data = deg2rad(deflection); 
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

