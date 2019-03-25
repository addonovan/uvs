#pragma once

#include <vector>

#include "units.hpp"

const Centimeter LIDAR_THRESHOLD = 100;
const double SONAR_THRESHOLD = -10.0; // [cm]

struct Reading {
    Radian angle;
    Centimeter range;
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
