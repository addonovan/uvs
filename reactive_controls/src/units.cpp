#include <cmath>
#include <stdexcept>

#include <units.hpp>

constexpr double DEGREE_TO_RADIAN_COEFFICIENT = PI / 180.0;
constexpr double RADIAN_TO_DEGREE_COEFFICIENT = 180.0 / PI;

//
// Radian
//

Radian::Radian() noexcept : NumericType(0.0) {}

Radian::Radian(double inner) : NumericType(inner) {
    if (!std::isfinite(inner)) {
        throw std::runtime_error{"Non-finite value of radians not permitted!"};
    }
}

Degree
Radian::as_degree() const noexcept {
    return Degree{m_inner * RADIAN_TO_DEGREE_COEFFICIENT};
}

//
// Degree
//

Degree::Degree(double inner) : NumericType(inner) {
    if (!std::isfinite(inner)) {
        throw std::runtime_error{"Non-finite value of degrees not permitted!"};
    }
}

Radian
Degree::as_radian() const noexcept {
    return Radian{m_inner * DEGREE_TO_RADIAN_COEFFICIENT};
}

//
// Centimeter
//

Centimeter::Centimeter(int inner) noexcept : NumericType(inner) {}

