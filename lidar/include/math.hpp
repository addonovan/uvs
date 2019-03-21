#pragma once

#include "utility.hpp"

struct Degree;
struct Radian;
struct Centimeter;

struct Degree final {
    public:
        Degree(double value);

        Degree operator+(const Degree& other) const;
        Degree operator-() const;
        
        Degree operator*(double scalar) const;
        Degree operator/(double scalar) const;

        bool operator<(const Degree& other) const;
        bool operator>(const Degree& other) const;
        bool operator==(const Degree& other) const;

        Radian as_radian() const;
        double as_double() const;

    private:
        double m_inner;
};

struct Radian final {
    public:
        Radian(double value);

        Radian operator+(const Radian& other) const;
        Radian operator-() const;

        Radian operator*(double scalar) const;
        Radian operator/(double scalar) const;

        bool operator<(const Radian& other) const;
        bool operator>(const Radian& other) const;
        bool operator==(const Radian& other) const;
        
        Degree as_degree() const;
        double as_double() const;

    private:
        double m_inner;
};

struct Centimeter final {
    public:
        Centimeter(int value);

        Centimeter operator+(const Centimeter& other) const;
        Centimeter operator-(const Centimeter& other) const;

        Centimeter operator*(int scalar) const;
        Centimeter operator/(int scalar) const;
        
        bool operator<(const Centimeter& other) const;
        bool operator>(const Centimeter& other) const;
        bool operator==(const Centimeter& other) const;
        
        int as_int() const;

    private:
        int m_inner;
};

/**
 * Converts the number in radians in the interval [-pi/2, pi/2] 
 * to degrees within the interval [-180, 180]. 
 *
 * This will also shift the angles around 180°, so that the 0° 
 * mark is straight forward.
 */
double rad2deg(double radians);

/**
 * Converts the degrees on the interval [-180, 180] to to the
 * interval [-pi/2, pi/2].
 */
double deg2rad(double degrees);

