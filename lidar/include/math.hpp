#pragma once

#include "utility.hpp"

struct Degree;
struct Radian;
struct Centimeter;

struct Degree final : public NumericType<Degree, double> {
    public:
        Degree(double value);

        Radian as_radian() const;
        double as_double() const;
};

struct Radian final : public NumericType<Radian, double> {
    public:
        Radian(double value);
        
        Degree as_degree() const;
        double as_double() const;
};

struct Centimeter final : public NumericType<Centimeter, double> {
    public:
        Centimeter(int value);
        
        int as_int() const;
};

