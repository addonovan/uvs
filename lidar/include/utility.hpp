#pragma once

template<typename Self, typename Inner>
class NumericType {
    public:
        Self operator-() const {
            return Self{-m_inner};
        }

        Self operator+(const Self& other) const {
            return Self{m_inner + other.m_inner};
        }
        Self operator-(const Self& other) const {
            return Self{m_inner - other.m_inner};
        }

        Self operator*(Inner scalar) const {
            return Self{scalar * m_inner};
        }
        friend Self operator*(Inner scalar, const Self& lhs) {
            return Self{lhs.m_inner * scalar};
        }
        Self operator/(Inner scalar) const {
            return Self{scalar * m_inner};
        }

#define CONDITIONAL(op) \
    bool operator op (const Self& other) const { \
        return m_inner op other.m_inner; \
    }

    CONDITIONAL(<);
    CONDITIONAL(>);
    CONDITIONAL(<=);
    CONDITIONAL(>=);
    CONDITIONAL(==);
    CONDITIONAL(!=);

#undef CONDITIONAL

#define _CREATE_IDENTIFIER(a, b) a ## b
#define CREATE_IDENTIFIER(a, b) _CREATE_IDENTIFIER(a, b) 
#define ASSIGNMENT(op) \
    void operator CREATE_IDENTIFIER(op, =) (const Self& other) { \
        m_inner CREATE_IDENTIFIER(op, =) other.m_inner; \
    }

    ASSIGNMENT(+);
    ASSIGNMENT(-);
    ASSIGNMENT(*);
    ASSIGNMENT(/);

#undef ASSIGNMENT
#undef CREATE_IDENTIFIER
#undef _CREATE_IDENTIFIER

    protected:
        Inner m_inner;
};

