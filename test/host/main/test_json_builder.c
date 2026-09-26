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
    // null-terminated JSON array containing only complete elements (never
    // a cut-off element) and must never overflow.
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
    // Not even one ~22-byte element fits in 20 bytes, so the result is the
    // empty array -- never a truncated element.
    TEST_ASSERT_EQUAL_STRING("[]", buf);
}

void test_json_builder_cap_fits_zero_elements(void)
{
    // Just enough room for "[]" and a NUL, not for any element.
    char buf[3];
    memset(buf, 0xAA, sizeof(buf));
    float x[2] = {1, 2}, y[2] = {1, 2};

    size_t n = build_x_y_samples_json(buf, sizeof(buf), x, y, 2, 2);

    TEST_ASSERT_EQUAL_STRING("[]", buf);
    TEST_ASSERT_EQUAL_size_t(2, n);
}

void test_json_builder_cap_fits_one_element(void)
{
    char buf[28];
    memset(buf, 0xAA, sizeof(buf));
    float x[2] = {1.0f, 2.0f}, y[2] = {1.0f, 2.0f};

    // "[{\"x\":1.000,\"y\":1.000}]" is 24 bytes + NUL = 25; a second element
    // plus ']'+NUL would need 47, so only the first element fits in 28.
    size_t n = build_x_y_samples_json(buf, sizeof(buf), x, y, 2, 2);

    TEST_ASSERT_EQUAL_STRING("[{\"x\":1.000,\"y\":1.000}]", buf);
    TEST_ASSERT_EQUAL_size_t(strlen(buf), n);
}

void test_json_builder_cap_fits_two_elements(void)
{
    char buf[48];
    memset(buf, 0xAA, sizeof(buf));
    float x[3] = {1.0f, 2.0f, 3.0f}, y[3] = {1.0f, 2.0f, 3.0f};

    size_t n = build_x_y_samples_json(buf, sizeof(buf), x, y, 3, 3);

    TEST_ASSERT_EQUAL_STRING(
        "[{\"x\":1.000,\"y\":1.000},{\"x\":2.000,\"y\":2.000}]", buf);
    TEST_ASSERT_EQUAL_size_t(strlen(buf), n);
}

void test_json_builder_cap_too_small_for_brackets(void)
{
    // Not even "[]" + NUL (3 bytes) fits: must not touch buf beyond a
    // leading NUL, and must report 0 bytes written.
    char buf[2] = {(char)0xAA, (char)0xAA};

    size_t n = build_x_y_samples_json(buf, sizeof(buf), (float[]){1}, (float[]){1}, 1, 1);

    TEST_ASSERT_EQUAL_size_t(0, n);
    TEST_ASSERT_EQUAL_CHAR('\0', buf[0]);
}
