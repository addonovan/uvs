#include <sonar.hpp>

namespace sonar {

    Radian calculate_deflection(Centimeter range) {
        Radian deflection;

        if (range < THRESHOLD) {
            deflection = *(THRESHOLD - range) * (3 * PI / 4) / *THRESHOLD;
        }

        return deflection;
    }
}
