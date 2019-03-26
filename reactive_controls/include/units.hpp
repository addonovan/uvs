#pragma clang diagnostic push
#pragma ide diagnostic ignored "google-explicit-constructor"
#pragma once

#include <boost/math/constants/constants.hpp>

#include "numeric_types.hpp"

constexpr double PI = boost::math::constants::pi<double>();

struct Radian;
struct Degree;
struct Centimeter;

struct Radian final : NumericType<Radian, double>, PartialOrdering<Radian> {

  public:

    Radian() noexcept;

    Radian(double inner);

    Degree as_degree() const noexcept;

};

struct Degree final : NumericType<Degree, double>, PartialOrdering<Degree> {

  public:

    Degree(double inner);

    Radian as_radian() const noexcept;

};


struct Centimeter final : NumericType<Centimeter, int>,
    TotalOrdering<Centimeter> {

  public:

    Centimeter() noexcept;

    Centimeter(int inner) noexcept;

};

#pragma clang diagnostic pop