#pragma once

#include <vector>
#include <sensor_msgs/LaserScan.h>

struct SensorReading final {
    public:
        int distance;
        int angle;
};

std::vector<SensorReading> 
convert_readings(const sensor_msgs::LaserScan::ConstPtr& data);

