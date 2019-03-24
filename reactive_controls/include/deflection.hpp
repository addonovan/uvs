#pragma once

#include <vector>

const double LIDAR_THRESHOLD = 100.0; // [cm]
const double SONAR_THRESHOLD = -10.0; // [cm]

struct Reading {
    double angle;
    double range;
};

double get_angle(double min, double step, int i);

bool is_valid_reading(
    double range_min,
    double range_max,
    double range,
    double angle
);

Reading find_min_reading(
    double angle_min,
    double angle_step,
    double range_min,
    double range_max,
    const std::vector<float>& ranges
);

double calculate_deflection(const Reading& reading);
