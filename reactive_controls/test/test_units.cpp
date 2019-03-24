#include <gtest/gtest.h>

#include <numeric_types.hpp>

TEST(TestNumericTypes, additiveOperators) {
    struct TestType : NumericType<TestType, int> {
        explicit TestType(int inner) : NumericType(inner) {}
    };

    auto a = TestType{10};
    auto b = TestType{5};
    auto c = TestType{17};

    ASSERT_EQ(*(a + b), 15);
    ASSERT_EQ(*(a - b), 5);
    ASSERT_EQ(*(a + c), 27);
    ASSERT_EQ(*(a - c), -7);
}

TEST(TestNumericTypes, scalarOperators) {
    struct TestType : NumericType<TestType, int> {
        explicit TestType(int inner) : NumericType(inner) {}
    };

    auto a = TestType{25};
    auto b = TestType{38};

    ASSERT_EQ(*(a * 5), 125);
    ASSERT_EQ(*(b * 10), 380);

    ASSERT_EQ(*(a / 10), 2);
    ASSERT_EQ(*(b / 5), 7);
}

TEST(TestNumericTypes, partialOrdering) {
    struct TestType : NumericType<TestType, int>, PartialOrdering<TestType> {
        explicit TestType(int inner) : NumericType(inner) {}
    };

    auto a = TestType{12};
    auto b = TestType{78};

    ASSERT_TRUE(b > a);
    ASSERT_TRUE(a < b);

    ASSERT_FALSE(a > b);
    ASSERT_FALSE(b < a);
}

TEST(TestNumericTypes, totalOrdering) {
    struct TestType : NumericType<TestType, int>, TotalOrdering<TestType> {
        explicit TestType(int inner) : NumericType(inner) {}
    };

    auto a = TestType{12};
    auto b = TestType{78};
    auto c = TestType{12};

    ASSERT_TRUE(b >= a);
    ASSERT_TRUE(a <= b);

    ASSERT_FALSE(a >= b);
    ASSERT_FALSE(b <= a);

    ASSERT_TRUE(a == c);
    ASSERT_TRUE(a >= c);
    ASSERT_TRUE(a <= c);
}
