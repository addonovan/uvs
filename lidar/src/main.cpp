#include <ctime>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

#include <deflection.hpp>
#include <math.hpp>

ros::Publisher* publisher = nullptr;

void on_lidar_message(const sensor_msgs::LaserScan::ConstPtr& message) {
    using namespace std::chrono;
    using hrclock = high_resolution_clock;

#ifndef NDEBUG
    hrclock::time_point start = hrclock::now();
#endif

    Degree deflection = calculate_deflection(message);

#ifndef NDEBUG
    hrclock::time_point end = hrclock::now();

    // ambiguous call to operator- here, so we have to manually choose which
    // template we want to instantiate (this part is 100% my fault but whatever)
    auto calc_duration = duration_cast<duration<double>>(end - start);

    // if we took more than 1/4th of the time needed to complete a scan, then
    // we need to issue a warning because that's bad!
    double calculation_time = calc_duration.count();
    double max_allowable_time = message->scan_time / 4;
    if (calculation_time > max_allowable_time) {
        ROS_WARN(
                "Calculation time (%1.5lfs) exceeded maximum allowed time (%1.5fs)",
                calculation_time,
                max_allowable_time
        );
    }
#endif

    // for large angles we should report the changes we're making
    if (deflection >= 1.0 || deflection <= -1.0) {
        ROS_INFO("Deflecting by %f°", deflection.as_double());
    }

    // respond with a 64-bit float telling by how many radians we should deflect
    std_msgs::Float64 msg;
    msg.data = deflection.as_radian().as_double(); 
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

