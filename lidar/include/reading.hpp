#pragma once

#include <vector>
#include <sensor_msgs/LaserScan.h>

#include "math.hpp"

struct SensorReading final {
    public:
        SensorReading(Centimeter dist, Degree ang) : distance{dist}, angle{ang} {}

        Centimeter distance;
        Degree angle;
};

