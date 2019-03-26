#pragma once

#include "units.hpp"

namespace sonar {

const Centimeter THRESHOLD = 100;

Radian calculate_deflection(Centimeter range);

}

