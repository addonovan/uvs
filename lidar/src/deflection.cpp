#include <cassert>
#include <ros/ros.h>
#include <math.hpp>
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
 * - positive angles are to the rover's right
 *   negative angles are to the rover's left
 * - reading.distance is a int measuring centimeters
 * - reading.distance will not be below about 15 cm
 */
Degree calculate_deflection_component(const SensorReading& reading) {
    const int THRESHOLD = 200;
    if (reading.distance > THRESHOLD) {
        return 0.0;
    }

    // find out how severe of a turn we should be making
    double severity = ((THRESHOLD - reading.distance) << 2) * 0.002;

    // calculate the desired direction we should be heading in to avoid
    // this obstacle (opposite of the direction of the reading)
    int direction;
    if (reading.angle > 0) {
        direction = -1;
    } else {
        direction = 1;
    }

    return severity * direction;
}

double weight_for(const Radian& angle) {
    Degree deg_angle = angle.as_degree(); 

    // throw out any deg_angles behind the rover
    if (deg_angle > 90.0 || deg_angle < -90.0) {
        return 0.0;
    }
    
    // bring to interval [0, 1] where 
    //      deg_angle = 0 => weight = 1
    double weight = (90.0 - deg_angle.as_double()) / 90.0; 

    // decrease the weight of going to the left or right
    weight = pow(weight, 2);

    return weight;
}

Degree calculate_deflection(const sensor_msgs::LaserScan::ConstPtr& message) {
    Radian angle_end = message->angle_max;
    Radian angle_step = message->angle_increment;
    double min_distance = message->range_min;
    double max_distance = message->range_max;

    // accumulators for a weighted average
    // (ax + by + cz) / (a + b + c) => weighted average of x, y, z
    // \____________/                  deflection accumulator
    //                  \_________/    weight accumulator
    Degree deflection_accumulator = 0.0;
    double weight_accumulator = 0.0;

    Radian angle = message->angle_min;
    for (int i = 0; angle < angle_end; i++, angle += angle_step) {
        // skip over invalid readings
        double distance = message->ranges[i];
        if (distance < min_distance 
         || distance > max_distance
         || !std::isfinite(distance)) {
            continue;
        } 
        
        // from experimentation 0.20 range is about 15cm
        SensorReading reading{
                static_cast<int>(distance * 15.0 / 0.20),
                angle.as_degree()
        };

        // calculate the deflection, and its weight
        double weight = weight_for(angle);
        Degree deflection = calculate_deflection_component(reading);
        deflection_accumulator += deflection * weight;
        weight_accumulator += weight; 
    }

    return deflection_accumulator / weight_accumulator;
}

