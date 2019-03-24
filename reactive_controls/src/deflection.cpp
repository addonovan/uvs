#include <cmath>
#include <cassert>

#include <deflection.hpp>

const double PI = 3.1416; // rounded up for comparisons
const double HALF_PI = PI / 2;
const double TAU = 2 * PI;

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

Reading find_min_reading(
    double angle_min,
    double angle_step,
    double range_min,
    double range_max,
    const std::vector<float>& ranges
) {
    Reading min_reading{1.0 / 0.0, 1.0 / 0.0};

    // iterate through all ranges, keeping track of its position (because we
    // need it to keep track of which angle we're at)
    int i = 0;
    for (double range : ranges) {
        double angle = get_angle(angle_min, angle_step, i++);

        // skip invalid readings
        if (!is_valid_reading(range_min, range_max, range, angle)) {
            continue;
        }

        // record only the minimum range along with the angle that generated it
        if (range < min_reading.range) {
            min_reading.range = range;
            min_reading.angle = angle;
        }
    }

    min_reading.range *= 100.0; // convert [m] to [cm]
    return min_reading;
}

double calculate_deflection(const Reading& reading) {
    // converted from following matlab code:
    // if (~isempty(lidar_range) && lidar_range <= lidar_threshold)
    //      if lidar_angle >= 0
    //          theta_change = -(lidar_threshold-lidar_range)^2*3*pi/4/lidar_threshold^2;
    //      elseif lidar_angle<0
    //          theta_change = (lidar_threshold-lidar_range)^2*3*pi/4/lidar_threshold^2;
    //      end
    //      ...
    // end
    //
    // - abs(theta_change) is the same on both branches
    // - isempty(lidar_range) is for when there're no readings, but that's
    //   handled by the calling function (guaranteed by our assertions)

    assert(std::isfinite(reading.range));
    assert(std::isfinite(reading.angle));

    // ignore values over the threshold
    if (reading.range > LIDAR_THRESHOLD) {
        return 0.0;
    }

    // calculate the magnitude of the angle
    double mag = pow(LIDAR_THRESHOLD - reading.range, 1) * (3 * PI / 4) / pow(LIDAR_THRESHOLD, 1);

    // multiply by the direction of the angle
    return mag * (reading.angle >= 0 ? -1 : 1);
}
