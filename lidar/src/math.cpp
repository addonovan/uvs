#include <cassert>
#include <cmath>
#include <math.hpp>

double rad2deg(double radians) {
    double degrees = radians * 180 / 3.14159; // close enough

    // rotate the coordinate system so that 0 is the front of the robot
    // then positive should be ccw (i.e. the robot's left)
    // and negative should bw cw (i.e. the robot's right)

    if (degrees < 0) {
        degrees += 180;
    } else {
        degrees -= 180;
    }

    assert(degrees > -181.0);
    assert(degrees < 180.0);
    return degrees;
}

double deg2rad(double degrees) {
    return degrees * 3.14159 / 180;
}

