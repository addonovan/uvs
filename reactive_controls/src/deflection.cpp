#include <cmath>
#include <cassert>
#include <algorithm>

#include <units.hpp>
#include <deflection.hpp>

constexpr double HALF_PI = PI / 2;
constexpr double TAU = 2 * PI;

double get_angle(double min, double step, int i) {
    // ported from the following matlab code:
    // angles = (pi - AngleMin + (0:numReadings-1)' * AngleIncrement);
    // angles = -(angles - 2*pi*floor((angles+pi)/(2*pi)));
    //
    // in matlab `angles` is an array, but here we generate each value one
    // at a time using `i` (elementOf [0, numReadings-1]).
    // AngleIncrement => step
    // AngleMin => min

    double angle = PI - min + (step * i);
    angle = -(angle - TAU * std::floor((angle + PI)/TAU));
    return angle;
}

bool is_valid_reading(
    double range_min,
    double range_max,
    double range,
    double angle
) {
    // ported from the following matlab code:
    // validIdx = (angles >= -pi/2) & (angles <= pi/2);
    // ...
    // angles = angles(validIdx);

    // get rid of values the lidar has marked as faulty
    if (!std::isfinite(range)) return false;
    if (range < range_min) return false;
    if (range > range_max) return false;

    return angle >= -HALF_PI && angle <= HALF_PI;
}

std::vector<Reading> map_readings(
    double angle_min,
    double angle_step,
    double range_min,
    double range_max,
    const std::vector<float>& ranges
) {
    std::vector<Reading> output;
    output.reserve(ranges.size());

    int i = 0;
    for (double range: ranges) {
        Radian angle = get_angle(angle_min, angle_step, i++);

        if (!is_valid_reading(range_min, range_max, range, *angle)) {
            continue;
        }

        Centimeter range_cm = static_cast<int>(range * 100); // convert [m] to [cm]

        output.push_back(Reading{angle, range_cm});
    }

    return output;
}

Radian calculate_deflection(const std::vector<Reading>& readings) {
    if (readings.empty()) {
        return 0.0;
    }

    // find the minimum element
    const auto& min = *std::min_element(
        readings.begin(), readings.end(),
        [](const Reading& a, const Reading& b) {
            return a.range < b.range;
        }
    );

    // calculate deflection based off of that
    double deflection = 0.0;
    if (min.range < LIDAR_THRESHOLD) {
        deflection = *(LIDAR_THRESHOLD - min.range) * (3 * PI / 4) / *LIDAR_THRESHOLD;
        if (min.angle > 0) {
            deflection *= -1;
        }
    }
    return deflection;
}

