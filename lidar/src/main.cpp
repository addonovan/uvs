#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

const double PI = 3.1416; // rounded up for comparisons
const double HALF_PI = PI / 2;
const double TAU = 2 * PI;

struct Reading { double angle, range; };

double get_angle(double min, double step, int i) {
    double angle = PI - (min + (step * i));
    angle = -(angle - TAU * std::floor((angle + PI)/(2 * PI)));
    return angle;
}

double THRESHOLD = 1.00; // m
bool is_valid_reading(
        double range_min,
        double range_max,
        double range, 
        double angle
) {
    if (std::isnan(range)) return false;
    if (range < range_min) return false;
    if (range > range_max) return false;
    if (!std::isfinite(range)) return false;
    if (range < THRESHOLD) return false;

    return angle >= -HALF_PI && angle <= HALF_PI;
}

Reading find_min_reading(const sensor_msgs::LaserScan::ConstPtr& message) {
    double angle_min = message->angle_min;
    double angle_step = message->angle_increment;

    double range_min = message->range_min;
    double range_max = message->range_max;

    Reading min_reading;
    min_reading.range = 1.0 / 0.0;
    min_reading.angle = 1.0 / 0.0;

    int i = 0;
    for (const double range : message->ranges) {
        double angle = get_angle(angle_min, angle_step, i++); 

        if (!is_valid_reading(range_min, range_max, range, angle)) {
            continue;
        }

        if (range < min_reading.range) {
            min_reading.range = range;
            min_reading.angle = angle;
        }
    }

    return min_reading;
}

//
// ROS Stuff
//

ros::Publisher* angle_pub = nullptr;
ros::Publisher* range_pub = nullptr;

void on_lidar_message(const sensor_msgs::LaserScan::ConstPtr& message) {
    Reading reading = find_min_reading(message);

    std_msgs::Float64 angle;
    angle.data = reading.angle; 
    angle_pub->publish(angle);

    std_msgs::Float64 range;
    range.data = reading.range; 
    range_pub->publish(range);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar");
    ros::NodeHandle n;

    // set up the publisher
    ros::Publisher angle_pub = n.advertise<std_msgs::Float64>("min_lidar_angle", 1);
    ::angle_pub = &angle_pub;

    ros::Publisher range_pub = n.advertise<std_msgs::Float64>("min_lidar_range", 1);
    ::range_pub = &range_pub;

    // set up the subscriber
    ros::Subscriber sub = n.subscribe("scan", 1, on_lidar_message);

    // run endlessly
    ROS_INFO("Initialized lidar reactive control system");
    ros::spin();
    return 0;
}

