#pragma once

#include <vector>
#include <sensor_msgs/LaserScan.h>

#include "math.hpp"

struct SensorReading final {
    public:
        SensorReading(int dist, Degree ang) : distance{dist}, angle{ang} {}

        int distance;
        Degree angle;
};

