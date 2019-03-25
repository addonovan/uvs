#pragma once

#include <vector>

#include "units.hpp"

const Centimeter LIDAR_THRESHOLD = 100;
const Centimeter SONAR_THRESHOLD = -10;

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

std::vector<Reading> map_readings(
    double angle_min,
    double angle_step,
    double range_min,
    double range_max,
    const std::vector<float>& ranges
);

Radian calculate_deflection(const std::vector<Reading>& readings);

