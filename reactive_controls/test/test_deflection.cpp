#include <cstdio>
#include <random>
#include <chrono>

#include <gtest/gtest.h>

#include <units.hpp>
#include <deflection.hpp>

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

TEST(TestDeflection, findsSmallestReading) {
    double angle_min = 0.0;
    double angle_step = PI / 4;
    double range_min = 0.0;
    double range_max = 1.0 / 0.0;

    // supplied distances in [m], values returned in [cm]
    auto entries = std::vector<float>{{
        250.0f,         // 0
        1 / 0.0f,       // PI / 4
        125.0f,         // PI / 2
        50.0f           // 3PI / 4
    }};

    auto reading = find_min_reading(
        angle_min, angle_step,
        range_min, range_max,
        entries
    );
    ASSERT_FLOAT_EQ(*reading.range, 5000.0);
    ASSERT_FLOAT_EQ(*reading.angle, get_angle(angle_min, angle_step, 3));

    reading = find_min_reading(
        angle_min, angle_step,
        100.0, range_max,
        entries
    );
    ASSERT_FLOAT_EQ(*reading.range, 12500.0);
    ASSERT_FLOAT_EQ(*reading.angle, get_angle(angle_min, angle_step, 2));

    reading = find_min_reading(
        angle_min, angle_step,
        500.0, 500.1,
        entries
    );
}

TEST(TestDeflection, deflectionOutOfRange) {
    auto reading = Reading{0.0, LIDAR_THRESHOLD + 50};
    assert_kinda_equal(calculate_deflection(reading), 0.0);

    reading.range = LIDAR_THRESHOLD + 1;
    assert_kinda_equal(calculate_deflection(reading), 0.0);
}

TEST(TestDeflection, deflectionInRange) {
    auto reading = Reading{PI / 4, LIDAR_THRESHOLD / 2};
    assert_kinda_equal(calculate_deflection(reading), -1.1781);

    // the opposite angle should give the opposite value
    reading.angle *= -1;
    assert_kinda_equal(calculate_deflection(reading),  1.1781);

    // angles closer to zero should have higher deflection magnitudes
    reading.range = 0;
    assert_kinda_equal(calculate_deflection(reading),  2.3562);

    reading.angle *= -1;
    assert_kinda_equal(calculate_deflection(reading), -2.3562);
}

TEST(TestDeflection, deflectionProperties) {
    auto reading = Reading{0, 0};

    // as the angle increases, the value should always become more negative
    double previous_deflection = 1.0 / 0.0;
    while (reading.angle < PI / 2) {
        double deflection = calculate_deflection(reading);

        if (std::isfinite(previous_deflection)) {
            ASSERT_GE(previous_deflection, deflection);
        }

        previous_deflection = deflection;
        reading.angle += PI / 16.0;
    }

    // this is peek STL right here...
    using std::chrono::system_clock;
    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(system_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<double> distribution{0.0, PI / 2};

    // f(theta) = x => f(-theta) = -x
    for (int i = 0; i < 100; i++) {
        double angle = distribution(generator);

        reading.angle = angle;
        double a = calculate_deflection(reading);

        reading.angle = -angle;
        double b = calculate_deflection(reading);
        ASSERT_FLOAT_EQ(a, -b);
    }
}
