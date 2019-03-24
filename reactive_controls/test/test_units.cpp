#include <gtest/gtest.h>

#include <units.hpp>

TEST(TestUnitValues, CentimeterOperators) {
    auto a = Centimeter{10};
    auto b = Centimeter{5};
    auto c = Centimeter{17};

    ASSERT_EQ(*(a + b), 15);
    ASSERT_EQ(*(a - b), 5);

    ASSERT_EQ(*(a * 5), 50);
    ASSERT_EQ(*(a / 5), 2);

    a += c; ASSERT_EQ(*a, 27);
    a -= c; ASSERT_EQ(*a, 10);
    a *= 6; ASSERT_EQ(*a, 60);
    a /= 6; ASSERT_EQ(*a, 10);
}
