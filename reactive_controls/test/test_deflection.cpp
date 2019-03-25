#include <cstdio>
#include <random>
#include <chrono>

#include <gtest/gtest.h>

#include <units.hpp>
#include <lidar.hpp>

/** Compares floats for similarity out to 3 decimal places */
#define assert_kinda_equal(a, b) \
        do { \
            auto lhs = std::floor(a * 1000) / 1000; \
            auto rhs = std::floor(b * 1000) / 1000; \
            ASSERT_FLOAT_EQ(lhs, rhs); \
        } while (false);

TEST(TestDeflection, convertAngle) {
    // generated by the outputs of the function
    assert_kinda_equal(get_angle(0.0, PI / 180.0, 0  ), +3.141600000000000);
    assert_kinda_equal(get_angle(0.0, PI / 180.0, 45 ), +2.356201836602552);
    assert_kinda_equal(get_angle(0.0, PI / 180.0, 90 ), +1.570803673205103);
    assert_kinda_equal(get_angle(0.0, PI / 180.0, 135), +0.785405509807656);
    assert_kinda_equal(get_angle(0.0, PI / 180.0, 180), +0.000007346410206);
    assert_kinda_equal(get_angle(0.0, PI / 180.0, 225), -0.785390816987241);
    assert_kinda_equal(get_angle(0.0, PI / 180.0, 270), -1.570788980384690);
    assert_kinda_equal(get_angle(0.0, PI / 180.0, 315), -2.356187143782138);
}

TEST(TestDeflection, infiniteReadingsInvalid) {
    ASSERT_FALSE(is_valid_reading(0.0, 100.0,  1.0 / 0.0, 0.0));
    ASSERT_FALSE(is_valid_reading(0.0, 100.0, -1.0 / 0.0, 0.0));
    ASSERT_FALSE(is_valid_reading(0.0, 100.0,  0.0 / 0.0, 0.0));
}

TEST(TestDeflection, readingsWithinRange) {
    ASSERT_FALSE(is_valid_reading(0.0, 100.0,   -1.0, 0.0));
    ASSERT_FALSE(is_valid_reading(0.0, 100.0, 1000.0, 0.0));

    ASSERT_TRUE(is_valid_reading(0.0, 100.0, 75.0, 0.0));
    ASSERT_TRUE(is_valid_reading(0.0, 100.0, 50.0, 0.0));
    ASSERT_TRUE(is_valid_reading(0.0, 100.0, 25.0, 0.0));
}

TEST(TestDeflection, readingsWithinAngleRange) {
    ASSERT_FALSE(is_valid_reading(0.0, 100.0, 50.0, -PI));
    ASSERT_FALSE(is_valid_reading(0.0, 100.0, 50.0, -(PI / 2) - 0.05));

    ASSERT_TRUE(is_valid_reading(0.0, 100.0, 50.0, -(PI / 2) + 0.05));
    ASSERT_TRUE(is_valid_reading(0.0, 100.0, 50.0, -(PI / 4)));
    ASSERT_TRUE(is_valid_reading(0.0, 100.0, 50.0,  0));
    ASSERT_TRUE(is_valid_reading(0.0, 100.0, 50.0,  (PI / 4)));
    ASSERT_TRUE(is_valid_reading(0.0, 100.0, 50.0,  (PI / 2) - 0.05));

    ASSERT_FALSE(is_valid_reading(0.0, 100.0, 50.0,  PI));
    ASSERT_FALSE(is_valid_reading(0.0, 100.0, 50.0,  (PI / 2) + 0.05));

}

TEST(TestDeflection, deflectionOutOfRange) {
    auto readings = std::vector<Reading>{{
        Reading{0.0, LIDAR_THRESHOLD + 50}
    }};
    assert_kinda_equal(*calculate_deflection(readings), 0.0);

    readings[0].range = LIDAR_THRESHOLD + 1;
    assert_kinda_equal(*calculate_deflection(readings), 0.0);
}

TEST(TestDeflection, deflectionInRange) {
    auto readings = std::vector<Reading>{{
        Reading{PI / 4, LIDAR_THRESHOLD / 2}
    }};
    assert_kinda_equal(*calculate_deflection(readings), -1.1781);

    // the opposite angle should give the opposite value
    readings[0].angle *= -1;
    assert_kinda_equal(*calculate_deflection(readings),  1.1781);

    // angles closer to zero should have higher deflection magnitudes
    readings[0].range = 0;
    assert_kinda_equal(*calculate_deflection(readings),  2.3562);

    readings[0].angle *= -1;
    assert_kinda_equal(*calculate_deflection(readings), -2.3562);
}

TEST(TestDeflection, deflectionProperties) {
    auto readings = std::vector<Reading>{{
        Reading{0, 0}
    }};

    // as the angle increases, the value should always become more negative
    double previous_deflection = 1.0 / 0.0;
    while (readings[0].angle < PI / 2) {
        double deflection = *calculate_deflection(readings);

        if (std::isfinite(previous_deflection)) {
            ASSERT_GE(previous_deflection, deflection);
        }

        previous_deflection = deflection;
        readings[0].angle += PI / 16.0;
    }

    // this is peek STL right here...
    using std::chrono::system_clock;
    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(system_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<double> distribution{0.0, PI / 2};

    // f(theta) = x => f(-theta) = -x
    for (int i = 0; i < 100; i++) {
        double angle = distribution(generator);

        readings[0].angle = angle;
        double a = *calculate_deflection(readings);

        readings[0].angle = -angle;
        double b = *calculate_deflection(readings);
        ASSERT_FLOAT_EQ(a, -b);
    }
}
