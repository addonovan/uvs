#include <cmath>
#include <cassert>
#include <ros/ros.h>
#include <deflection.hpp>

/**
 * Calculate the deflection component derived from this one reading. This is 
 * applied to each and every (valid) reading that the sensor reads. Each
 * component is then combined and weighted with a function based on its angle,
 * thus these can all be knee-jerk reactions, but they will probably get 
 * smoothed out.
 *
 * Assumptions:
 * - reading.angle is an int within [-180, 180]
 * - reading.angle is 0.0 for readings directly in front of the rover 
 * - reading.distance is a int measuring centimeters
 * - reading.distance will not be below about 15 cm
 */
double calculate_deflection_component(const SensorReading& reading) {
    const int THRESHOLD = 200;

    if (reading.angle > -5 && reading.angle < 5 && reading.distance < THRESHOLD) {
        return ((THRESHOLD - reading.distance) << 2) * 0.00003;
    }

    return 0.0;
}

double rad2deg(double radians) {
    double degrees = radians * 180 / 3.14159; // close enough

    // rotate the coordinate system so that 0 is the front of the robot
    // then positive should be ccw (i.e. the robot's left)
    // and negative should bw cw (i.e. the robot's right)

    if (degrees < 0) {
        degrees += 180;
    } else {
        degrees -= 180;
    }

    assert(degrees > -181.0);
    assert(degrees < 180.0);
    return degrees;
}

double weight_for(int angle) {
    assert(angle > -180);
    assert(angle <= 180);

    // throw out any angles behind the rover
    if (angle > 90 || angle < -90) {
        return 0;
    }
    
    // bring to interval [0, 1] where 
    //      angle = 0 => weight = 1
    double weight = (90.0 - angle) / 90.0; 

    // decrease the weight of going to the left or right
    weight = pow(weight, 2);

    return weight;
}

double calculate_deflection(const sensor_msgs::LaserScan::ConstPtr& message) {
    double angle_end = message->angle_max;
    double angle_step = message->angle_increment;
    double min_distance = message->range_min;
    double max_distance = message->range_max;

    // accumulators for a weighted average
    // (ax + by + cz) / (a + b + c) => weighted average of x, y, z
    // \____________/                  deflection accumulator
    //                  \_________/    weight accumulator
    double deflection_accumulator = 0.0;
    double weight_accumulator = 0.0;

    double angle = message->angle_min;
    for (int i = 0; angle < angle_end; i++, angle += angle_step) {
        // skip over invalid readings
        double distance = message->ranges[i];
        if (distance < min_distance 
         || distance > max_distance
         || !std::isfinite(distance)) {
            continue;
        } 
        
        // from experimentation 0.20 range is about 15cm
        SensorReading reading;
        reading.distance = static_cast<int>(distance * 15 / 0.20);
        reading.angle = static_cast<int>(rad2deg(angle));

        // calculate the deflection, and its weight
        double weight = weight_for(angle);
        double deflection = calculate_deflection_component(reading);
        deflection_accumulator += weight * deflection;
        weight_accumulator += weight; 
    }

    return deflection_accumulator / weight_accumulator;
}

