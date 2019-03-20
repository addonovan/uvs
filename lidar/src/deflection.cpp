#include <deflection.hpp>

double calculate_deflection_component(const SensorReading& reading) {
    const double THRESHOLD = 200;
    if (reading.angle < 1.0 && reading.distance < THRESHOLD) {
        return pow(THRESHOLD - reading.distance, 2) * 0.00003;
    }

    return 0.0;
}

double calculate_deflection(const sensor_msgs::LaserScan::ConstPtr& message) {
    double angle_end = message->angle_max;
    double angle_step = message->angle_increment;
    double min_distance = message->range_min;
    double max_distance = message->range_max;

    int reading_count = 0;
    double deflection_accumulator = 0.0;

    double angle = message->angle_min;
    for (int i = 0; angle < angle_end; i++, angle += angle_step) {
        SensorReading reading;
        reading.distance = message->ranges[i];
        reading.angle = angle;

        // skip over invalid readings
        if (reading.distance < min_distance || reading.distance > max_distance) {
            continue;
        } 

        deflection_accumulator += calculate_deflection_component(reading);
        reading_count++;
    }

    // return the average deflection component
    if (reading_count == 0) {
        return 0.0;
    } else {
        return deflection_accumulator / reading_count;
    }
}
