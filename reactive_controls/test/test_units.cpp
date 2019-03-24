#include <gtest/gtest.h>

#include <units.hpp>

TEST(TestUnitValues, CentimeterOperators) {
    auto a = Centimeter{10};
    auto b = Centimeter{5};

    ASSERT_EQ(*(a + b), 15);
    ASSERT_EQ(*(a - b), 5);

    ASSERT_EQ(*(a * 5), 50);
    ASSERT_EQ(*(a / 5), 2);

    a += b; ASSERT_EQ(*a, 15);
    a -= b; ASSERT_EQ(*a, 10);
    a *= 5; ASSERT_EQ(*a, 50);
    a /= 5; ASSERT_EQ(*a, 10);
}
