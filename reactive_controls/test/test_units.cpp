#include <type_traits>
#include <stdexcept>
#include <climits>

#include <gtest/gtest.h>

#include <units.hpp>

using limit = std::numeric_limits<double>;

TEST(TestUnits, centimeterProperties) {
    bool is_numeric_type = std::is_base_of<
        NumericType<Centimeter, int>,
        Centimeter
    >::value;
    bool is_partially_orderable = std::is_base_of<
        PartialOrdering<Centimeter>,
        Centimeter
    >::value;
    bool is_totally_orderable = std::is_base_of<
        TotalOrdering<Centimeter>,
        Centimeter
    >::value;

    ASSERT_TRUE(is_numeric_type);
    ASSERT_TRUE(is_partially_orderable);
    ASSERT_TRUE(is_totally_orderable);
}

TEST(TestUnits, radianProperties) {
    bool is_numeric_type = std::is_base_of<
        NumericType<Radian, double>,
        Radian
    >::value;
    bool is_partially_orderable = std::is_base_of<
        PartialOrdering<Radian>,
        Radian
    >::value;
    bool is_totally_orderable = std::is_base_of<
        TotalOrdering<Radian>,
        Radian
    >::value;

    ASSERT_TRUE(is_numeric_type);
    ASSERT_TRUE(is_partially_orderable);
    ASSERT_FALSE(is_totally_orderable);
}

TEST(TestUnits, degreeProperties) {
    bool is_numeric_type = std::is_base_of<
        NumericType<Degree, double>,
        Degree
    >::value;
    bool is_partially_orderable = std::is_base_of<
        PartialOrdering<Degree>,
        Degree
    >::value;
    bool is_totally_orderable = std::is_base_of<
        TotalOrdering<Degree>,
        Degree
    >::value;

    ASSERT_TRUE(is_numeric_type);
    ASSERT_TRUE(is_partially_orderable);
    ASSERT_FALSE(is_totally_orderable);
}

TEST(TestUnits, radianRejectsNonfiniteValues) {
    try {
        Radian{limit::infinity()};
        ADD_FAILURE() << "Radian accepted an infinite value";
    } catch (...) {}

    try {
        Radian{limit::quiet_NaN()};
        ADD_FAILURE() << "Radian accepted a quiet NaN value";
    } catch (...) {}

    try {
        Radian{limit::signaling_NaN()};
        ADD_FAILURE() << "Radian accepted a signaling NaN value";
    } catch (...) {}
}

TEST(TestUnits, degreeRejectsNonfiniteValues) {
    try {
        Degree{limit::infinity()};
        ADD_FAILURE() << "Degree accepted an infinite value";
    } catch (...) {}

    try {
        Degree{limit::quiet_NaN()};
        ADD_FAILURE() << "Degree accepted a quiet NaN value";
    } catch (...) {}

    try {
        Degree{limit::signaling_NaN()};
        ADD_FAILURE() << "Degree accepted a signaling NaN value";
    } catch (...) {}
}

TEST(TestUnits, angleConversions) {
    const Degree MAX_ALLOWABLE_DEGREE_DELTA = 0.01;
    const Radian MAX_ALLOWABLE_RADIAN_DELTA = MAX_ALLOWABLE_DEGREE_DELTA.as_radian();

    auto a = Degree{90};
    auto b = Radian{PI / 2.0};

    ASSERT_TRUE(a.as_radian() - MAX_ALLOWABLE_RADIAN_DELTA < b);
    ASSERT_TRUE(a.as_radian() + MAX_ALLOWABLE_RADIAN_DELTA > b);

    ASSERT_TRUE(b.as_degree() - MAX_ALLOWABLE_DEGREE_DELTA < a);
    ASSERT_TRUE(b.as_degree() + MAX_ALLOWABLE_DEGREE_DELTA > a);
}
