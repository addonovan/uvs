#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

double calculate_angle(int distance) {
    const double THRESHOLD = 200;
    if (distance < THRESHOLD) {
        return pow(THRESHOLD - distance, 2) * 0.00003;
    }

    return 0;
}

//
// Boilerplate ROS
//

using Output = std_msgs::Float64;
using Input = sensor_msgs::LaserScan;

ros::Publisher* publisher = nullptr;

void on_lidar_message(const Input::ConstPtr& message) {
    ROS_INFO("Range 0: %lf", message->ranges[0]);

    Output msg;
    msg.data = calculate_angle(message->ranges[0]); 
    publisher->publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar");
    ros::NodeHandle n;

    // set up the publisher
    ros::Publisher pub = n.advertise<Output>("delta_commanded_theta", 1);
    publisher = &pub;

    // set up the subscriber
    ros::Subscriber sub = n.subscribe("scan", 1, on_lidar_message);

    // run endlessly
    ros::spin();
    return 0;
}

