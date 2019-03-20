#pragma once

#include "reading.hpp"

double calculate_deflection_component(const SensorReading& reading);
double rad2deg(double radians);
double calculate_deflection(const sensor_msgs::LaserScan::ConstPtr& message);

