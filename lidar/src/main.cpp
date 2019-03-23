#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

const double PI = 3.1416; // rounded up for comparisons
const double HALF_PI = PI / 2;
const double TAU = 2 * PI;

const double THRESHOLD = 1.00; // [m]

struct Reading { double angle, range; };

double get_angle(double min, double step, int i) {
    // ported from the following matlab code:
    // angles = (pi - AngleMin + (0:numReadings-1)' * AngleIncrement);
    // angles = -(angles - 2*pi*floor((angles+pi)/(2*pi)));
    // 
    // in matlab `angles` is an array, but here we generate each value one
    // at a time using `i` (elementOf [0, numReadings-1]).
    // AngleIncrement => step
    // AngleMin => min

    double angle = PI - (min + (step * i));
    angle = -(angle - TAU * std::floor((angle + PI)/TAU));
    return angle;
}

bool is_valid_reading(
        double range_min,
        double range_max,
        double range, 
        double angle
) {
    // get rid of values the lidar has marked as faulty
    if (!std::isfinite(range)) return false;
    if (range < range_min) return false;
    if (range > range_max) return false;

    return angle >= -HALF_PI && angle <= HALF_PI;
}

Reading find_min_reading(const sensor_msgs::LaserScan::ConstPtr& message) {
    double angle_min = message->angle_min;
    double angle_step = message->angle_increment;

    double range_min = message->range_min;
    double range_max = message->range_max;

    Reading min_reading;
    min_reading.range = 1.0 / 0.0; // generates an inf
    min_reading.angle = 1.0 / 0.0;

    // iterate through all ranges, keeping track of its position (because we
    // need it to keep track of which angle we're at)
    int i = 0;
    for (const double range : message->ranges) {
        double angle = get_angle(angle_min, angle_step, i++); 

        // skip invalid readings
        if (!is_valid_reading(range_min, range_max, range, angle)) {
            continue;
        }

        // record only the minimum range along with the angle that generated it
        if (range < min_reading.range) {
            min_reading.range = range;
            min_reading.angle = angle;
        }
    }

    return min_reading;
}

double calculate_deflection(Reading reading) {
    // function is guaranteed to only be called when an actual min reading
    // is found
    assert(std::isfinite(reading.range));  
    assert(std::isfinite(reading.angle));
  
    // ignore values over the threshold 
    if (reading.range > THRESHOLD) {
        return 0.0; 
    }

    // calculate the magnitude of the angle
    double mag = pow(THRESHOLD - reading.range, 2) * (3 * PI / 4) / pow(THRESHOLD, 2);

    // multiply by the direction of the angle
    return mag * (reading.angle >= 0 ? -1 : 1);
}

//
// ROS Stuff
//

ros::Publisher* angle_pub = nullptr;
ros::Publisher* range_pub = nullptr;
ros::Publisher* dflxn_pub = nullptr;

void on_lidar_message(const sensor_msgs::LaserScan::ConstPtr& message) {
    Reading reading = find_min_reading(message);

    std_msgs::Float64 angle;
    angle.data = reading.angle; 
    angle_pub->publish(angle);

    std_msgs::Float64 range;
    range.data = reading.range; 
    range_pub->publish(range);
    
    double deflection = 0.0;
    if (std::isfinite(reading.range)) {
        deflection = calculate_deflection(reading);
    }
    std_msgs::Float64 dflxn;
    dflxn.data = deflection; 
    dflxn_pub->publish(dflxn);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar");
    ros::NodeHandle n;

    // set up the publisher
    ros::Publisher angle_pub = n.advertise<std_msgs::Float64>("min_lidar_angle", 1);
    ::angle_pub = &angle_pub;

    ros::Publisher range_pub = n.advertise<std_msgs::Float64>("min_lidar_range", 1);
    ::range_pub = &range_pub;

    ros::Publisher dflxn_pub = n.advertise<std_msgs::Float64>("theta_deflection", 1);
    ::dflxn_pub = &dflxn_pub;

    // set up the subscriber
    ros::Subscriber sub = n.subscribe("scan", 1, on_lidar_message);

    // run endlessly
    ROS_INFO("Initialized lidar reactive control system");
    ros::spin();
    return 0;
}

