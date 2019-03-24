#include <cmath>
#include <mutex>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>

#include <units.hpp>

const double PI = 3.1416; // rounded up for comparisons
const double HALF_PI = PI / 2;
const double TAU = 2 * PI;

const double LIDAR_THRESHOLD = 100.0; // [cm]
const double SONAR_THRESHOLD = -10.0; // [cm]

//
// Deflection Publisher
//

ros::Publisher* publisher = nullptr;
std::mutex mtx_publisher;

double lidar_deflection = 0.0 / 0.0;
double sonar_deflection = 0.0 / 0.0;

template<bool IsSonar>
void publish_deflection(double deflection) {
    std::lock_guard<std::mutex> lock{mtx_publisher};
    std_msgs::Float64 message;

    if (IsSonar) {
        sonar_deflection = deflection;
    } else {
        lidar_deflection = deflection;
    }

    if (sonar_deflection > 0.0) {
        message.data = sonar_deflection;
    } else if (lidar_deflection > 0.0) {
        message.data = lidar_deflection;
    } else {
        message.data = 0.0;
    }
    
    publisher->publish(message);
}

//
// Deflection Calculations
//

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

    double angle = PI - min + (step * i);
    angle = -(angle - TAU * std::floor((angle + PI)/TAU));
    return angle;
}

bool is_valid_reading(
        double range_min,
        double range_max,
        double range, 
        double angle
) {
    // ported from the following matlab code:
    // validIdx = (angles >= -pi/2) & (angles <= pi/2);
    // ... 
    // angles = angles(validIdx);

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
    for (double range : message->ranges) {
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

    min_reading.range *= 100.0; // convert [m] to [cm]
    return min_reading;
}

double calculate_deflection(Reading reading) {
    // converted from following matlab code:
    // if (~isempty(lidar_range) && lidar_range <= lidar_threshold)
    //      if lidar_angle >= 0
    //          theta_change = -(lidar_threshold-lidar_range)^2*3*pi/4/lidar_threshold^2;
    //      elseif lidar_angle<0
    //          theta_change = (lidar_threshold-lidar_range)^2*3*pi/4/lidar_threshold^2;
    //      end
    //      ...
    // end
    //
    // - abs(theta_change) is the same on both branches
    // - isempty(lidar_range) is for when there're no readings, but that's 
    //   handled by the calling function (guaranteed by our assertions)

    assert(std::isfinite(reading.range));  
    assert(std::isfinite(reading.angle));

    // ignore values over the threshold 
    if (reading.range > LIDAR_THRESHOLD) {
        return 0.0; 
    }

    // calculate the magnitude of the angle
    double mag = pow(LIDAR_THRESHOLD - reading.range, 1) * (3 * PI / 4) / pow(LIDAR_THRESHOLD, 1);

    // multiply by the direction of the angle
    return mag * (reading.angle >= 0 ? -1 : 1);
}

//
// ROS Stuff
//

void on_lidar_message(const sensor_msgs::LaserScan::ConstPtr& message) {
    Reading reading = find_min_reading(message);

    // publish no deflection, unless we actually find a valid reading and
    // then we'll publish that deflection 
    double deflection = 0.0;
    if (std::isfinite(reading.range)) {
        deflection = calculate_deflection(reading);
    }
    if (abs(deflection) > 0.0009) {
        ROS_INFO(
                "LIDAR Deflection: %+1.3lf [rad]  Obstacle @ %3.0lf [cm] @ %+1.3lf [rad]", 
                deflection, 
                reading.range, 
                reading.angle
        );
    }

    publish_deflection<false>(deflection);    
}

void on_sonar_message(const std_msgs::UInt16::ConstPtr& message) {
    double range = message->data / 10.0; // convert [mm] to [cm]

    double deflection = 0.0;
    if (range < SONAR_THRESHOLD) {
        deflection = pow(SONAR_THRESHOLD - range, 1) * (3 * PI / 4) / pow(SONAR_THRESHOLD, 1);
        ROS_INFO("SONAR Deflection: %+1.3lf [rad]  Obstacle @ %3.0lf [cm]", deflection, range);
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

