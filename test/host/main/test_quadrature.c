#include "unity.h"

#include "quadrature.h"

// One full CW quadrature cycle per the QDEC_TABLE convention in
// quadrature.c: 00 -> 10 -> 11 -> 01 -> 00 (clk<<1|dt). levels are
// {clk, dt} pairs.
static const int cw_cycle[4][2] = {
    {1, 0},
    {1, 1},
    {0, 1},
    {0, 0},
};

// The reverse (CCW) cycle.
static const int ccw_cycle[4][2] = {
    {0, 1},
    {1, 1},
    {1, 0},
    {0, 0},
};

void test_quadrature_cw_sequence_emits_one_step_per_cycle(void)
{
    qdec_t q;
    qdec_init(&q, 0, 0);

    int total = 0;
    for (int cycle = 0; cycle < 3; cycle++)
    {
        for (int i = 0; i < 4; i++)
        {
            total += qdec_update(&q, cw_cycle[i][0], cw_cycle[i][1], 4);
        }
    }
    TEST_ASSERT_EQUAL_INT(3, total);
}

void test_quadrature_ccw_sequence_emits_negative_one_step_per_cycle(void)
{
    qdec_t q;
    qdec_init(&q, 0, 0);

    int total = 0;
    for (int cycle = 0; cycle < 3; cycle++)
    {
        for (int i = 0; i < 4; i++)
        {
            total += qdec_update(&q, ccw_cycle[i][0], ccw_cycle[i][1], 4);
        }
    }
    TEST_ASSERT_EQUAL_INT(-3, total);
}

void test_quadrature_bounce_emits_nothing(void)
{
    qdec_t q;
    qdec_init(&q, 0, 0);

    // clk toggling back and forth with dt steady: 00 -> 10 -> 00 -> 10 -> 00.
    // Each of these is a valid single-bit transition in the table, but they
    // cancel out (the accumulator moves +/-1 and back) so no full step is
    // ever emitted at counts_per_step=4.
    int steps_emitted = 0;
    for (int i = 0; i < 20; i++)
    {
        int clk = (i % 2 == 0) ? 0 : 1;
        int step = qdec_update(&q, clk, 0, 4);
        if (step != 0)
        {
            steps_emitted++;
        }
    }
    TEST_ASSERT_EQUAL_INT(0, steps_emitted);
}

void test_quadrature_invalid_jump_ignored(void)
{
    qdec_t q;
    qdec_init(&q, 0, 0);

    // 00 -> 11 is a two-bit jump: not reachable in one legal quadrature step,
    // so the table must treat it as invalid (delta 0, no accumulation).
    int step = qdec_update(&q, 1, 1, 4);
    TEST_ASSERT_EQUAL_INT(0, step);

    // Confirm the decoder still works normally afterwards (state resynced to
    // whatever was actually read).
    int total = 0;
    for (int i = 0; i < 4; i++)
    {
        total += qdec_update(&q, cw_cycle[i][0], cw_cycle[i][1], 4);
    }
    TEST_ASSERT_TRUE(total == 0 || total == 1);
}
