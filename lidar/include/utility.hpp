#pragma once

template<typename T, typename U>
void operator+=(T& lhs, const U& rhs) {
    lhs = lhs + rhs;
}

template<typename T, typename U>
void operator-=(T& lhs, const U& rhs) {
    lhs = lhs - rhs;
}

template<typename T, typename U>
void operator*=(T& lhs, const U& rhs) {
    lhs = lhs * rhs;
}

template<typename T, typename U>
void operator/=(T& lhs, const U& rhs) {
    lhs = lhs / rhs;
}
