#pragma clang diagnostic push
#pragma ide diagnostic ignored "google-explicit-constructor"
#pragma once

#include "numeric_types.hpp"

struct Centimeter final : NumericType<Centimeter, int>,
    TotalOrdering<Centimeter> {

  public:

    Centimeter(int inner) noexcept : NumericType(inner) {}

};

#pragma clang diagnostic pop