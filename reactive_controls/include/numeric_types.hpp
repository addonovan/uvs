#pragma once

#include <algorithm>

template<class Self, typename Inner>
class NumericType {

    //
    // Constructors
    //

  public:

    NumericType(Inner inner) : m_inner{inner} {}

    //
    // Operators
    //

  public:

    Self operator-() const { return Self{-m_inner}; }

    Self operator+(const Self& rhs) const { return Self{m_inner + rhs.m_inner}; }
    Self operator-(const Self& rhs) const { return Self{m_inner - rhs.m_inner}; }
    void operator+=(const Self& rhs) { *this = *this + rhs; }
    void operator-=(const Self& rhs) { *this = *this - rhs; }

    Self operator*(Inner rhs) const { return Self{m_inner * rhs}; }
    Self operator/(Inner rhs) const { return Self{m_inner / rhs}; }
    void operator*=(Inner rhs) { *this = *this * rhs; }
    void operator/=(Inner rhs) { *this = *this / rhs; }

    Inner operator*() const { return m_inner; }

    //
    // Members
    //

  protected:

    Inner m_inner;

};

template<class Self>
class PartialOrdering {

    bool operator<(const Self& other) const noexcept {
        return static_cast<Self>(this)->m_inner < other.m_inner;
    }

    bool operator>(const Self& other) const noexcept {
        return static_cast<Self>(this)->m_inner > other.m_inner;
    }

};

template<class Self>
class TotalOrdering : PartialOrdering<Self> {

    bool operator==(const Self& other) const noexcept {
        return static_cast<Self>(this)->m_inner < other.m_inner;
    }

    bool operator<=(const Self& other) const noexcept {
        return *this < other || *this == other;
    }

    bool operator>=(const Self& other) const noexcept {
        return *this > other || *this == other;
    }

};
