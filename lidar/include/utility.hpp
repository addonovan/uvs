#pragma once

template<typename T>
T operator-(const T& lhs, const T& rhs) {
    return lhs + -rhs;
}

#define COMMUTATIVITY(op) \
    template<typename T, typename U>\
    U operator op (const T& lhs, const U& rhs) { return rhs * lhs; }

COMMUTATIVITY(+);
COMMUTATIVITY(*);

#undef COMMUTATIVITY

#define _CREATE_IDENTIFIER(a, b) a ## b
#define CREATE_IDENTIFIER(a, b) _CREATE_IDENTIFIER(a, b) 
#define ASSIGNMENT(op) \
    template<typename T, typename U> \
    void operator CREATE_IDENTIFIER(op, =) (T& lhs, const U& rhs) { \
        lhs = lhs op rhs; \
    }

ASSIGNMENT(+);
ASSIGNMENT(-);
ASSIGNMENT(*);
ASSIGNMENT(/);

#undef ASSIGNMENT
#undef CREATE_IDENTIFIER
#undef _CREATE_IDENTIFIER

template<typename T, typename U>
bool operator<=(const T& lhs, const U& rhs) {
    return lhs < rhs || lhs == rhs;
}

template<typename T, typename U>
bool operator>=(const T& lhs, const U& rhs) {
    return lhs > rhs || lhs == rhs;
}

