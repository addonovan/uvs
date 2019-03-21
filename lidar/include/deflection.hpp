#pragma once

#include "reading.hpp"
#include "math.hpp"

Degree calculate_deflection_component(const SensorReading& reading);

Degree calculate_deflection(const sensor_msgs::LaserScan::ConstPtr& message);

