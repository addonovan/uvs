#include <cassert>
#include <cmath>
#include <math.hpp>

constexpr double PI = 3.14159;
constexpr double RADIAN_TO_DEGREE_FACTOR = 180 / PI;

//
// Degree Implementation
//

Degree::Degree(double value) {
    auto integral_part = static_cast<long long int>(value);
    auto fraction_part = value - integral_part;
    m_inner = (integral_part % 180) + fraction_part;

    assert(value >= -180);
    assert(value <= 180);
}

Radian Degree::as_radian() const { 
    return Radian{m_inner / RADIAN_TO_DEGREE_FACTOR};
}
double Degree::as_double() const { 
    return m_inner;
}
Degree Degree::as_robot_coordinates() const {
    double value = m_inner;
    if (value < 0.0) {
        value += 180.0;
    } else {
        value -= 180.0;
    }

    assert(value > -181.0);
    assert(value < 180.0);
    return Degree{value};
}

//
// Radian Implementation 
//

Radian::Radian(double value) {
    auto integral_part = static_cast<long long int>(value);
    auto fraction_part = value - integral_part;
    
    integral_part %= 180;
    m_inner = integral_part + fraction_part;

    assert(value >= -PI);
    assert(value <= PI);
}

Degree Radian::as_degree() const { 
    return Degree{m_inner * RADIAN_TO_DEGREE_FACTOR};
}
double Radian::as_double() const { return m_inner; }

//
// Centimeter Implementation
//

Centimeter::Centimeter(int value) {
    assert(value >= 0);
    m_inner = value;
}

int Centimeter::as_int() const { 
    return m_inner;
}

