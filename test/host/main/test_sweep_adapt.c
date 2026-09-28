#include <math.h>
#include <stdint.h>

#include "unity.h"
#include "sweep_plan.h"

#define N_POINTS 20
#define PWM_RES 8191u
#define SCALE_MA_PER_DUTY 0.479f // hardware: load current per duty step
#define VOC_MV 20000.0f          // synthetic panel Voc, 20 V
#define INA_OFFSET_MA 3.0f       // simulated INA219 zero-current offset error

// Deterministic, small (~+-1.5%) pseudo-noise so the model isn't perfectly
// linear, without making the test flaky.
static float jitter(uint32_t duty)
{
    uint32_t h = duty * 2654435761u;
    float frac = (float)(h % 1000) / 1000.0f; // 0..1
    return 1.0f + (frac - 0.5f) * 0.03f;       // 0.985..1.015
}

// Single-diode-ish simulated panel, same shape as test_sweep_range.c's
// panel_probe(), but driven by the electronic load's actual duty->current
// law: commanded_mA = max(0, duty*SCALE_MA_PER_DUTY - deadzone_mA), with an
// INA219-like additive offset error folded into the reading (measurement.c
// is responsible for subtracting it out before it ever reaches
// sweep_adapt_next(); this simulation just exercises that the arc-length
// stepper is not thrown off by the residual noise it leaves behind).
// Knee sharpness of the simulated panel (logistic steepness around Isc).
static double s_knee_sharpness = 8.0;

static void panel_at_duty(float isc_mA, float deadzone_mA, uint32_t duty,
                           float *out_v_mV, float *out_i_mA)
{
    double commanded_mA = (double)duty * SCALE_MA_PER_DUTY - (double)deadzone_mA;
    if (commanded_mA < 0.0)
        commanded_mA = 0.0;

    double overdrive = commanded_mA / (double)isc_mA;
    double v = (double)VOC_MV / (1.0 + exp(s_knee_sharpness * (overdrive - 1.0)));
    double i = commanded_mA < isc_mA ? commanded_mA : isc_mA;
    i *= jitter(duty);
    i += INA_OFFSET_MA * 0.05; // small residual after offset correction

    if (v < 0.0)
        v = 0.0;
    if (i < 0.0)
        i = 0.0;

    *out_v_mV = (float)v;
    *out_i_mA = (float)i;
}

static uint32_t hard_max(void)
{
    uint32_t hm = PWM_RES * SWEEP_DUTY_MAX_PERCENT / 100;
    return hm < 1 ? 1 : hm;
}

typedef struct
{
    uint32_t duties[N_POINTS];
    float v_mV[N_POINTS];
    float i_mA[N_POINTS];
    int count;
    bool reached_isc;
} sweep_result_t;

// Drives sweep_adapt_next() against the simulated panel, starting from the
// Voc probe (duty 0), the same way producer_task does.
static sweep_result_t run_adapt(float isc_mA, float deadzone_mA)
{
    uint32_t knee = (uint32_t)(isc_mA / SCALE_MA_PER_DUTY) + (uint32_t)(deadzone_mA / SCALE_MA_PER_DUTY);
    if (knee > hard_max())
        knee = hard_max();
    uint32_t top = (uint32_t)(((uint64_t)knee * SWEEP_KNEE_HEADROOM_PERCENT) / 100);
    if (top > hard_max())
        top = hard_max();
    if (top < N_POINTS)
        top = N_POINTS;

    sweep_adapt_t a;
    sweep_adapt_begin(&a, VOC_MV, isc_mA, knee, top, N_POINTS);

    sweep_result_t res = {0};
    uint32_t duty = 0;
    float v_mV, i_mA;
    panel_at_duty(isc_mA, deadzone_mA, duty, &v_mV, &i_mA);

    for (int iter = 0; iter < N_POINTS + SWEEP_ADAPT_REFINE_POINTS + 2 && res.count < N_POINTS; iter++)
    {
        res.duties[res.count] = duty;
        res.v_mV[res.count] = v_mV;
        res.i_mA[res.count] = i_mA;
        res.count++;

        uint32_t next = sweep_adapt_next(&a, v_mV, i_mA);

        if (sweep_adapt_last_discarded(&a))
        {
            // Jumped over the knee: the point is dropped and the knee is
            // re-swept from below (the caller drains the load at duty 0).
            res.count--;
            TEST_ASSERT_TRUE_MESSAGE(next > res.duties[res.count - 1], "re-sweep did not start above the last point");
            duty = next;
            panel_at_duty(isc_mA, deadzone_mA, duty, &v_mV, &i_mA);
            continue;
        }

        if (v_mV <= VOC_MV * SWEEP_ADAPT_COLLAPSE_PERCENT / 100.0f && duty > 0)
            res.reached_isc = true;

        if (next == SWEEP_ADAPT_DONE)
            break;

        TEST_ASSERT_TRUE_MESSAGE(next > duty, "duty did not strictly increase");
        TEST_ASSERT_TRUE_MESSAGE(next <= top, "duty exceeded top");

        duty = next;
        panel_at_duty(isc_mA, deadzone_mA, duty, &v_mV, &i_mA);
    }

    return res;
}

static void assert_common_invariants(const sweep_result_t *res, float isc_mA)
{
    TEST_ASSERT_TRUE_MESSAGE(res->count <= N_POINTS, "produced more points than the budget");

    for (int i = 1; i < res->count; i++)
    {
        TEST_ASSERT_TRUE_MESSAGE(res->duties[i] > res->duties[i - 1], "duties not strictly increasing");
    }

    // Coverage: with a large enough Isc, at least 4 points should sit in
    // the steep low-current region (I < 0.7*Isc) and at least 4 in the
    // flat near-Isc region (I >= 0.7*Isc), i.e. the arc-length placement
    // actually spreads across the whole curve rather than bunching at the
    // knee.
    if (isc_mA >= 10.0f)
    {
        int low = 0, high = 0;
        for (int i = 0; i < res->count; i++)
        {
            if (res->i_mA[i] < 0.7f * isc_mA)
                low++;
            else
                high++;
        }
        TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(4, low, "not enough points in the steep/low-current region");
        TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(4, high, "not enough points in the flat/near-Isc region");
    }
}

void test_sweep_adapt_reaches_isc_across_range(void)
{
    // Duty resolution floor: ~0.479 mA per duty step, so at Isc == 3 mA the
    // whole curve collapses within about 6 duty steps of the knee -- there
    // just isn't enough duty resolution to resolve much shape. We still
    // require the sweep to terminate cleanly and stay within budget.
    float isc_values[] = {3, 10, 50, 200, 700};
    for (size_t k = 0; k < sizeof(isc_values) / sizeof(isc_values[0]); k++)
    {
        float isc_mA = isc_values[k];
        for (int dz = 0; dz <= 1; dz++)
        {
            float deadzone_mA = dz ? 3.0f : 0.0f;
            sweep_result_t res = run_adapt(isc_mA, deadzone_mA);

            assert_common_invariants(&res, isc_mA);
            // Termination is valid either because V collapsed (Isc reached)
            // or because duty reached this sweep's own top -- top itself is
            // derived from the knee with headroom, so ending there without
            // a full 3%-of-Voc collapse is expected behavior, not a bug.
            uint32_t knee = (uint32_t)(isc_mA / SCALE_MA_PER_DUTY) + (uint32_t)(deadzone_mA / SCALE_MA_PER_DUTY);
            if (knee > hard_max())
                knee = hard_max();
            uint32_t top = (uint32_t)(((uint64_t)knee * SWEEP_KNEE_HEADROOM_PERCENT) / 100);
            if (top > hard_max())
                top = hard_max();
            if (top < N_POINTS)
                top = N_POINTS;
            TEST_ASSERT_TRUE_MESSAGE(res.reached_isc || res.duties[res.count - 1] >= top,
                                      "sweep neither reached Isc nor hit its own top");
        }
    }
}

void test_sweep_adapt_no_large_normalized_gaps(void)
{
    // No normalized (v,i) gap between consecutive points should exceed
    // about 3x the target per-step arc length, except where the duty
    // resolution floor (~0.5 mA/step) makes a finer step physically
    // impossible -- that only bites at very small Isc, so this is checked
    // at Isc = 50 mA where duty resolution is not the limiting factor.
    float isc_mA = 50.0f;
    sweep_result_t res = run_adapt(isc_mA, 0.0f);

    float ds_target = SWEEP_ADAPT_TARGET_ARC / (float)(N_POINTS - 1);

    for (int i = 1; i < res.count; i++)
    {
        float dv = (res.v_mV[i - 1] - res.v_mV[i]) / VOC_MV;
        float di = (res.i_mA[i] - res.i_mA[i - 1]) / isc_mA;
        float ds = sqrtf(dv * dv + di * di);
        TEST_ASSERT_TRUE_MESSAGE(ds <= ds_target * 3.0f, "normalized gap between points too large");
    }
}

void test_sweep_adapt_tiny_isc_terminates_within_budget(void)
{
    // At the duty-resolution floor (Isc a couple of duty steps wide),
    // coverage buckets don't apply, but the sweep must still terminate
    // within the point budget and never exceed top.
    sweep_result_t res = run_adapt(3.0f, 0.0f);
    TEST_ASSERT_TRUE(res.count >= 1);
    TEST_ASSERT_TRUE(res.count <= N_POINTS);
}

void test_sweep_adapt_prints_isc_50ma_and_5ma_tables(void)
{
    // Not a real assertion-bearing test beyond "it runs"; prints the two
    // tables the task report asks for via TEST_MESSAGE so `./host_tests.elf`
    // output captures them.
    float isc_values[] = {50.0f, 5.0f};
    for (size_t k = 0; k < 2; k++)
    {
        sweep_result_t res = run_adapt(isc_values[k], 0.0f);
        char line[96];
        snprintf(line, sizeof(line), "-- Isc=%.0f mA table (duty, I mA, V mV) --", (double)isc_values[k]);
        TEST_MESSAGE(line);
        for (int i = 0; i < res.count; i++)
        {
            snprintf(line, sizeof(line), "duty=%-5lu I=%-8.2f V=%-8.1f",
                     (unsigned long)res.duties[i], (double)res.i_mA[i], (double)res.v_mV[i]);
            TEST_MESSAGE(line);
        }
    }
    TEST_ASSERT_TRUE(true);
}

void test_sweep_adapt_resweeps_a_sharp_knee(void)
{
    // Real panel on the bench (Isc ~50 mA): V fell from ~17 V to ~0 V within
    // two duty steps, so the first pass jumped straight over the knee. The
    // stepper must drop that collapse point, re-sweep the gap from below and
    // still end on the collapse with strictly increasing recorded duties.
    s_knee_sharpness = 60.0;
    sweep_result_t res = run_adapt(50.0f, 0.0f);
    s_knee_sharpness = 8.0;

    int knee_points = 0;
    for (int k = 0; k < res.count; k++)
    {
        if (k > 0)
            TEST_ASSERT_TRUE_MESSAGE(res.duties[k] > res.duties[k - 1], "recorded duties not strictly increasing");
        if (res.v_mV[k] > 0.1f * VOC_MV && res.v_mV[k] < 0.9f * VOC_MV)
            knee_points++;
    }
    TEST_ASSERT_TRUE_MESSAGE(res.reached_isc, "sharp-knee sweep did not end on the collapse");
    TEST_ASSERT_TRUE_MESSAGE(knee_points >= 2, "no points on the sharp knee");
}
