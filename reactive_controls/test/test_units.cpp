#include <type_traits>

#include <gtest/gtest.h>

#include <units.hpp>

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
