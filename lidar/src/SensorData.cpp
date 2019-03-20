#include <SensorData.hpp>

std::vector<SensorReading> 
convert_readings(const sensor_msgs::LaserScan::ConstPtr& data) {
    auto output = std::vector<SensorReading>{};

    double angle_end = data->angle_max;
    double angle_step = data->angle_increment;

    double angle = data->angle_min;
    for (int i = 0; angle < angle_end; i++, angle += angle_step) {
        SensorReading reading;
        reading.distance = data->ranges[i];
        reading.angle = angle;
        output.push_back(reading);
    }

    return output;
}

