#include <cmath>
#include <stdexcept>

#include <units.hpp>

//
// Radian
//

Radian::Radian(double inner) : NumericType(inner) {
    if (!std::isfinite(inner)) {
        throw std::runtime_error{"Non-finite value of radians not permitted!"};
    }
}

//
// Degree
//

Degree::Degree(double inner) : NumericType(inner) {
    if (!std::isfinite(inner)) {
        throw std::runtime_error{"Non-finite value of degrees not permitted!"};
    }
}

//
// Centimeter
//

Centimeter::Centimeter(int inner) noexcept : NumericType(inner) {}

