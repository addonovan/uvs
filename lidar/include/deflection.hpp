#pragma once

#include "reading.hpp"

double rad2deg(double radians);
double deg2rad(double degrees);

double calculate_deflection_component(const SensorReading& reading);
double calculate_deflection(const sensor_msgs::LaserScan::ConstPtr& message);

