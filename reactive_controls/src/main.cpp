#include <cmath>
#include <mutex>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>

#include <units.hpp>
#include <lidar.hpp>
#include <sonar.hpp>

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

    // prefer to deflect by sonar rather than lidar
    if (sonar_deflection > 0.0) {
        message.data = *sonar_deflection;
    } else if (lidar_deflection > 0.0) {
        message.data = *lidar_deflection;
    } else {
        message.data = 0.0;
    }

#define FORMAT " deflection by %+1.3lf rad"
    if (abs(message.data) > 0.0009) {
        if (IsSonar) {
            ROS_INFO("SONAR" FORMAT, message.data);
        } else {
            ROS_INFO("LIDAR" FORMAT, message.data);
        }
    }
#undef FORMAT

    publisher->publish(message);
}

void on_lidar_message(const sensor_msgs::LaserScan::ConstPtr& message) {
    using namespace lidar;

    auto readings = map_readings(
        message->angle_min,
        message->angle_increment,
        message->range_min,
        message->range_max,
        message->ranges
    );

    Radian deflection = calculate_deflection(readings);
    publish_deflection<false>(deflection);
}

void on_sonar_message(const std_msgs::UInt16::ConstPtr& message) {
    using namespace sonar;

    Centimeter range = message->data / 10; // convert [mm] to [cm]

    Radian deflection = calculate_deflection(range);
    publish_deflection<true>(deflection);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "reactive_controls");
    ros::NodeHandle n;

    // set up the publisher
    ros::Publisher publisher = n.advertise<std_msgs::Float64>("theta_deflection", 1);
    ::publisher = &publisher;

    // set up the subscriber
    ros::Subscriber lidar_subscriber = n.subscribe("scan", 1, on_lidar_message);
    ros::Subscriber sonar_subscriber = n.subscribe("sonar", 1, on_sonar_message);

    // run endlessly
    ROS_INFO("Initialized reactive control system");
    ros::spin();

    return 0;
}

