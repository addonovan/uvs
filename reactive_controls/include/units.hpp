#pragma clang diagnostic push
#pragma ide diagnostic ignored "google-explicit-constructor"
#pragma once

#include "numeric_types.hpp"

struct Radian final : NumericType<Radian, double>, PartialOrdering<Radian> {

  public:

    Radian(double inner);

};

struct Degree final : NumericType<Degree, double>, PartialOrdering<Degree> {

  public:

    Degree(double inner);

};


struct Centimeter final : NumericType<Centimeter, int>,
    TotalOrdering<Centimeter> {

  public:

    Centimeter(int inner) noexcept;

};

#pragma clang diagnostic pop