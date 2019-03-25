#include <sonar.hpp>

namespace sonar {

    Radian calculate_deflection(Centimeter range) {
        Radian deflection;

        if (range < SONAR_THRESHOLD) {
            deflection = *(SONAR_THRESHOLD - range) * (3 * PI / 4) / *SONAR_THRESHOLD;
        }

        return deflection;
    }
}
