#include "unity.h"

#include "chart_math.h"

void test_chart_mpp_index_finds_max_power(void)
{
    float v[] = {0, 5, 10, 15, 18, 20};
    float i[] = {50, 50, 49, 45, 20, 0};
    TEST_ASSERT_EQUAL_INT(3, chart_mpp_index(v, i, 6));
    TEST_ASSERT_EQUAL_INT(0, chart_mpp_index(v, i, 0));
}

void test_chart_scale_maps_and_clamps(void)
{
    TEST_ASSERT_EQUAL_INT(0, chart_scale(0.0f, 20.0f, 100));
    TEST_ASSERT_EQUAL_INT(99, chart_scale(20.0f, 20.0f, 100));
    TEST_ASSERT_EQUAL_INT(99, chart_scale(30.0f, 20.0f, 100));
    TEST_ASSERT_EQUAL_INT(0, chart_scale(-1.0f, 20.0f, 100));
    TEST_ASSERT_EQUAL_INT(0, chart_scale(5.0f, 0.0f, 100));
}

void test_chart_double_press_window(void)
{
    TEST_ASSERT_TRUE(chart_is_double_press(1000, 1300, 400));
    TEST_ASSERT_FALSE(chart_is_double_press(1000, 1500, 400));
    TEST_ASSERT_TRUE(chart_is_double_press(0xFFFFFF00u, 0x20u, 400));
}
