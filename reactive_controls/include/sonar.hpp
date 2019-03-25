#pragma once

#include "units.hpp"

namespace sonar {

const Centimeter SONAR_THRESHOLD = -10;

Radian calculate_deflection(Centimeter range);

}

