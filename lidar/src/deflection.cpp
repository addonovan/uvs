#include <cassert>
#include <ros/ros.h>
#include <math.hpp>
#include <deflection.hpp>

static Centimeter THRESHOLD{200};

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
    if (reading.distance > THRESHOLD) {
        return 0.0;
    }

    // calculate the weight of the angle at this point in time [0, 1]
    auto thresh = THRESHOLD.as_int();
    double weight = thresh - reading.distance.as_int();
    weight /= thresh;

    // try to go 90° away from the problem direction
    Degree direction = reading.angle >= 0 ?
        reading.angle - Degree{90.0} : Degree{90.0} - reading.angle;

    // scale that angle down by the weight (if the object is further away,
    // we don't need to take as drastic measures to avoid it as if it were
    // closer)
    return weight * direction;
}

double weight_for(const Degree& angle) {
    // throw out any angles behind the rover
    if (angle > 90.0 || angle < -90.0) {
        return 0.0;
    }
    
    // bring to interval [0, 1] where 
    //      angle = 0 => weight = 1
    double weight = (90.0 - angle.as_double()) / 90.0; 

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
        double range = message->ranges[i];
        if (range < min_distance 
         || range > max_distance
         || !std::isfinite(range)) {
            continue;
        } 

        Centimeter distance = static_cast<int>(range * 15.0 / 0.20);
        Degree angle_deg = angle.as_degree();
        
        // from experimentation 0.20 range is about 15cm
        SensorReading reading{distance, angle_deg};

        // calculate the deflection, and its weight
        double weight = weight_for(angle_deg);
        Degree deflection = calculate_deflection_component(reading);
        deflection_accumulator += deflection * weight;
        weight_accumulator += weight; 
    }

    return deflection_accumulator / weight_accumulator;
}

