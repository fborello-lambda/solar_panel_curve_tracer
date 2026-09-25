#include <stdint.h>

#include "unity.h"
#include "sweep_plan.h"

#define N_POINTS 20
#define PWM_RES 8191u
#define SCALE_MA_PER_DUTY 0.479f // synthetic panel: mA per duty step

static uint32_t hard_max(void)
{
    uint32_t hm = PWM_RES * SWEEP_DUTY_MAX_PERCENT / 100;
    return hm < 1 ? 1 : hm;
}

static uint32_t knee_for_isc(float isc_mA)
{
    uint32_t hm = hard_max();
    uint32_t duty_for_isc = (uint32_t)(isc_mA / SCALE_MA_PER_DUTY);
    return (duty_for_isc > hm) ? hm : duty_for_isc;
}

static uint32_t top_for_knee(uint32_t knee)
{
    uint32_t hm = hard_max();
    uint32_t top = (uint32_t)(((uint64_t)knee * SWEEP_KNEE_HEADROOM_PERCENT) / 100);
    if (top > hm)
        top = hm;
    if (top < N_POINTS)
        top = N_POINTS;
    return top;
}

static void assert_valid_plan(uint32_t top, uint32_t knee)
{
    uint32_t duties[N_POINTS];
    sweep_plan_build(top, knee, duties, N_POINTS);

    TEST_ASSERT_EQUAL_UINT32(0, duties[0]);
    TEST_ASSERT_EQUAL_UINT32(top, duties[N_POINTS - 1]);

    for (int i = 0; i < N_POINTS; i++)
    {
        TEST_ASSERT_TRUE_MESSAGE(duties[i] <= hard_max(), "duty exceeds hard cap");
        if (i > 0)
        {
            TEST_ASSERT_TRUE_MESSAGE(duties[i] > duties[i - 1], "duties not strictly increasing");
        }
    }
}

void test_sweep_plan_build_shape_across_isc_range(void)
{
    float isc_values_mA[] = {2, 5, 10, 20, 50, 100, 150, 200, 300, 400, 500, 600, 700, 800};
    for (size_t k = 0; k < sizeof(isc_values_mA) / sizeof(isc_values_mA[0]); k++)
    {
        uint32_t knee = knee_for_isc(isc_values_mA[k]);
        uint32_t top = top_for_knee(knee);
        assert_valid_plan(top, knee);
    }
}

void test_sweep_plan_build_never_collapsed_case(void)
{
    // Mirrors auto_range's never-collapsed fallback: top = hard_max,
    // knee = hard_max * 100 / SWEEP_KNEE_HEADROOM_PERCENT.
    uint32_t top = hard_max();
    uint32_t knee = hard_max() * 100 / SWEEP_KNEE_HEADROOM_PERCENT;
    assert_valid_plan(top, knee);
}

void test_sweep_plan_build_knee_clustering(void)
{
    // At Isc=50mA the fine leg should cluster points tightly around the
    // knee duty: at least 10 of the 20 points must land within
    // 85%..102% of the knee duty.
    uint32_t knee = knee_for_isc(50.0f);
    uint32_t top = top_for_knee(knee);

    uint32_t duties[N_POINTS];
    sweep_plan_build(top, knee, duties, N_POINTS);

    uint32_t band_lo = knee * 85 / 100;
    uint32_t band_hi = knee * 102 / 100;

    int in_band = 0;
    for (int i = 0; i < N_POINTS; i++)
    {
        if (duties[i] >= band_lo && duties[i] <= band_hi)
            in_band++;
    }

    TEST_ASSERT_GREATER_OR_EQUAL_INT(10, in_band);
}
