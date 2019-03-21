#include <cassert>
#include <cmath>
#include <math.hpp>

constexpr double PI = 3.14159;
constexpr double RADIAN_TO_DEGREE_FACTOR = 180 / PI;

//
// Degree Implementation
//

Degree::Degree(int value) : m_inner{value % 180} {
    assert(value >= -180);
    assert(value <= 180);
}

Degree Degree::operator+(const Degree& other) const { 
    return Degree{m_inner + other.m_inner}; 
}
Degree Degree::operator-(const Degree& other) const { 
    return Degree{m_inner - other.m_inner}; 
}

Degree Degree::operator*(double scalar) const { 
    return Degree{static_cast<int>(scalar * m_inner)}; 
}
Degree Degree::operator/(double scalar) const { 
    return Degree{static_cast<int>(scalar / m_inner)}; 
}

bool Degree::operator<(const Degree& other) const {
    return m_inner < other.m_inner;
}
bool Degree::operator>(const Degree& other) const {
    return m_inner > other.m_inner;
}
bool Degree::operator==(const Degree& other) const {
    return m_inner == other.m_inner;
}

Radian Degree::as_radian() const { 
    return Radian{m_inner / RADIAN_TO_DEGREE_FACTOR};
}
int Degree::as_int() const { 
    return m_inner;
}

//
// Radian Implementation 
//

Radian::Radian(double value) : m_inner{value} {
    auto integral_part = static_cast<long long int>(value);
    auto fraction_part = value - integral_part;
    
    integral_part %= 180;
    m_inner = integral_part + fraction_part;

    assert(value >= -PI);
    assert(value <= PI);
}

Radian Radian::operator+(const Radian& other) const {
    return Radian{m_inner + other.m_inner};
}
Radian Radian::operator-(const Radian& other) const {
    return Radian{m_inner - other.m_inner};
}

Radian Radian::operator*(double scalar) const {
    return Radian{m_inner * scalar};
}
Radian Radian::operator/(double scalar) const {
    return Radian{m_inner / scalar};
}

Degree Radian::as_degree() const { 
    return Degree{static_cast<int>(m_inner * RADIAN_TO_DEGREE_FACTOR)};
}
double Radian::as_double() const { return m_inner; }

//
// Conversion
//

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

