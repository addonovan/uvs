#pragma once

template<typename T>
T operator-(const T& lhs, const T& rhs) {
    return lhs + -rhs;
}

template<typename T, typename U>
U operator*(const T& lhs, const U& rhs) {
    return rhs * lhs;
}

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

template<typename T, typename U>
bool operator<=(const T& lhs, const U& rhs) {
    return lhs < rhs || lhs == rhs;
}

template<typename T, typename U>
bool operator>=(const T& lhs, const U& rhs) {
    return lhs > rhs || lhs == rhs;
}

