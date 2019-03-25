#include <cmath>
#include <mutex>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>

#include <units.hpp>
#include <deflection.hpp>

//
// Deflection Publisher
//

ros::Publisher* publisher = nullptr;
std::mutex mtx_publisher;

Radian lidar_deflection;
Radian sonar_deflection;

template<bool IsSonar>
void publish_deflection(Radian deflection) {
    std::lock_guard<std::mutex> lock{mtx_publisher};
    std_msgs::Float64 message;

    if (IsSonar) {
        sonar_deflection = deflection;
    } else {
        lidar_deflection = deflection;
    }

    if (sonar_deflection > 0.0) {
        message.data = *sonar_deflection;
    } else if (lidar_deflection > 0.0) {
        message.data = *lidar_deflection;
    } else {
        message.data = 0.0;
    }
    
    publisher->publish(message);
}

//
// ROS Stuff
//

void on_lidar_message(const sensor_msgs::LaserScan::ConstPtr& message) {
    auto readings = map_readings(
        message->angle_min,
        message->angle_increment,
        message->range_min,
        message->range_max,
        message->ranges
    );

    // publish no deflection, unless we actually find a valid reading and
    // then we'll publish that deflection
    Radian deflection = calculate_deflection(readings);
    if (abs(*deflection) > 0.0009) {
        ROS_INFO("LIDAR Deflection: %+1.3lf [rad]", *deflection);
    }

    publish_deflection<false>(deflection);
}

void on_sonar_message(const std_msgs::UInt16::ConstPtr& message) {
    Centimeter range = static_cast<int>(message->data / 10); // convert [mm] to [cm]

    Radian deflection;

    if (range < SONAR_THRESHOLD) {
        deflection = *(SONAR_THRESHOLD - range) * (3 * PI / 4) / *SONAR_THRESHOLD;
        ROS_INFO("SONAR Deflection: %+1.3lf [rad]", *deflection);
    }
    
    publish_deflection<true>(deflection);    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar");
    ros::NodeHandle n;

    // set up the publisher
    ros::Publisher publisher = n.advertise<std_msgs::Float64>("theta_deflection", 1);
    ::publisher = &publisher;

    // set up the subscriber
    ros::Subscriber lidar_subscriber = n.subscribe("scan", 1, on_lidar_message);
    ros::Subscriber sonar_subscriber = n.subscribe("sonar", 1, on_sonar_message);

    // run endlessly
    ROS_INFO("Initialized lidar reactive control system");
    ros::spin();
    return 0;
}

