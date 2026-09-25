#include <string.h>

#include "unity.h"
#include "json_builder.h"

void test_json_builder_zero_points(void)
{
    char buf[64] = "unwritten";
    float x[1] = {1.0f}, y[1] = {2.0f};

    size_t n = build_x_y_samples_json(buf, sizeof(buf), x, y, 1, 0);

    TEST_ASSERT_EQUAL_size_t(0, n);
    TEST_ASSERT_EQUAL_STRING("unwritten", buf); // buffer untouched on count<=0
}

void test_json_builder_one_point(void)
{
    char buf[64];
    float x[1] = {22.464f}, y[1] = {36.0f};

    size_t n = build_x_y_samples_json(buf, sizeof(buf), x, y, 1, 1);

    TEST_ASSERT_EQUAL_STRING("[{\"x\":22.464,\"y\":36.000}]", buf);
    TEST_ASSERT_EQUAL_size_t(strlen(buf), n);
}

void test_json_builder_n_points(void)
{
    char buf[256];
    float x[3] = {0.0f, 1.5f, -2.25f};
    float y[3] = {10.0f, 20.0f, 30.0f};

    size_t n = build_x_y_samples_json(buf, sizeof(buf), x, y, 3, 3);

    TEST_ASSERT_EQUAL_STRING(
        "[{\"x\":0.000,\"y\":10.000},{\"x\":1.500,\"y\":20.000},{\"x\":-2.250,\"y\":30.000}]", buf);
    TEST_ASSERT_EQUAL_size_t(strlen(buf), n);

    // count > max_samples is clamped to max_samples.
    size_t n2 = build_x_y_samples_json(buf, sizeof(buf), x, y, 2, 3);
    TEST_ASSERT_EQUAL_STRING("[{\"x\":0.000,\"y\":10.000},{\"x\":1.500,\"y\":20.000}]", buf);
    TEST_ASSERT_EQUAL_size_t(strlen(buf), n2);
}

void test_json_builder_truncates_small_buffer(void)
{
    // A buffer too small to hold every point must still produce a valid,
    // null-terminated JSON array (truncated, not garbage) and never
    // overflow.
    char buf[20];
    memset(buf, 0xAA, sizeof(buf));
    float x[5] = {1, 2, 3, 4, 5};
    float y[5] = {1, 2, 3, 4, 5};

    size_t n = build_x_y_samples_json(buf, sizeof(buf), x, y, 5, 5);

    TEST_ASSERT_TRUE(n < sizeof(buf));
    TEST_ASSERT_EQUAL_size_t(strlen(buf), n);
    TEST_ASSERT_EQUAL_CHAR('\0', buf[n]);
    TEST_ASSERT_EQUAL_CHAR(']', buf[n - 1]);
    TEST_ASSERT_EQUAL_CHAR('[', buf[0]);
}
