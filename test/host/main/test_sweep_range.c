#include <math.h>
#include <stdint.h>

#include "unity.h"
#include "sweep_plan.h"

#define PWM_RES 8191u
#define SCALE_MA_PER_DUTY 0.479f // synthetic panel: mA per duty step at low duty
#define VOC_MV 20000.0f          // synthetic panel Voc, 20 V

// Deterministic, small (~+-1.5%) pseudo-noise so the model isn't perfectly
// linear, without making the test flaky.
static float jitter(uint32_t duty)
{
    uint32_t h = duty * 2654435761u;
    float frac = (float)(h % 1000) / 1000.0f; // 0..1
    return 1.0f + (frac - 0.5f) * 0.03f;       // 0.985..1.015
}

// Single-diode-ish simulated panel: current is a capped ramp, voltage is a
// smooth logistic that sits near Voc while under Isc and collapses toward 0
// once the commanded current would exceed Isc.
static void panel_probe(float isc_mA, uint32_t duty, float *out_v_mV, float *out_i_mA)
{
    double commanded_mA = (double)duty * SCALE_MA_PER_DUTY;
    double overdrive = commanded_mA / (double)isc_mA;

    double v = (double)VOC_MV / (1.0 + exp(8.0 * (overdrive - 1.0)));
    double i = commanded_mA < isc_mA ? commanded_mA : isc_mA;
    i *= jitter(duty);

    if (v < 0.0)
        v = 0.0;
    if (i < 0.0)
        i = 0.0;

    *out_v_mV = (float)v;
    *out_i_mA = (float)i;
}

// Drives sweep_range through a full auto-range sequence against the
// simulated panel. Returns the finished sweep_range_t and asserts probes
// were strictly ascending and never exceeded hard_max.
static sweep_range_t run_auto_range(float isc_mA, float voc_mV)
{
    sweep_range_t r;
    sweep_range_begin(&r, PWM_RES, 20);

    bool has_panel = sweep_range_on_voc(&r, voc_mV);
    if (!has_panel)
        return r;

    uint32_t prev_duty = 0;
    int guard = 0;
    for (;;)
    {
        uint32_t duty = r.duty;
        TEST_ASSERT_TRUE_MESSAGE(duty > prev_duty, "probe duty did not strictly ascend");
        TEST_ASSERT_TRUE_MESSAGE(duty <= r.hard_max, "probe duty exceeded hard_max");

        float v_mV, i_mA;
        panel_probe(isc_mA, duty, &v_mV, &i_mA);

        prev_duty = duty;
        bool done = sweep_range_on_probe(&r, v_mV, i_mA);
        guard++;
        TEST_ASSERT_TRUE_MESSAGE(guard < 64, "auto-range never finished");
        if (done)
            break;
    }
    return r;
}

void test_sweep_range_probes_strictly_ascending_and_bounded(void)
{
    // The ascending/bounded assertions live inside run_auto_range() and
    // fire for every probe, so simply running it across a spread of Isc
    // values exercises them.
    float isc_values[] = {5, 50, 200, 600};
    for (size_t k = 0; k < sizeof(isc_values) / sizeof(isc_values[0]); k++)
    {
        sweep_range_t r = run_auto_range(isc_values[k], VOC_MV);
        TEST_ASSERT_EQUAL(SWEEP_RANGE_DONE, r.status);
    }
}

void test_sweep_range_knee_estimate_tracks_isc(void)
{
    float isc_values[] = {5, 50, 200, 600};
    for (size_t k = 0; k < sizeof(isc_values) / sizeof(isc_values[0]); k++)
    {
        float isc_mA = isc_values[k];
        sweep_range_t r = run_auto_range(isc_mA, VOC_MV);

        TEST_ASSERT_EQUAL(SWEEP_RANGE_DONE, r.status);
        TEST_ASSERT_TRUE(r.collapsed);

        float expected_knee = isc_mA / SCALE_MA_PER_DUTY;
        float rel_err = fabsf((float)r.knee - expected_knee) / expected_knee;
        TEST_ASSERT_TRUE_MESSAGE(rel_err <= 0.10f, "knee duty not within 10% of Isc/scale");
    }
}

void test_sweep_range_low_voc_is_no_panel(void)
{
    sweep_range_t r;
    sweep_range_begin(&r, PWM_RES, 20);

    bool has_panel = sweep_range_on_voc(&r, 400.0f); // below SWEEP_VOC_MIN_MV (500)
    TEST_ASSERT_FALSE(has_panel);
    TEST_ASSERT_EQUAL(SWEEP_RANGE_NO_PANEL, r.status);
}

void test_sweep_range_strong_panel_hits_hard_max(void)
{
    // Isc so large the panel never collapses before the duty cap:
    // duty_for_isc = 5000/0.479 ~= 10438, well above hard_max (~1638).
    sweep_range_t r = run_auto_range(5000.0f, VOC_MV);

    TEST_ASSERT_EQUAL(SWEEP_RANGE_DONE, r.status);
    TEST_ASSERT_FALSE(r.collapsed);
    TEST_ASSERT_EQUAL_UINT32(r.hard_max, r.top);
}

void test_sweep_range_collapse_on_first_probe_is_sane(void)
{
    // Force a collapse on the very first loaded probe (duty ==
    // SWEEP_PROBE_START_DUTY): sweep_range must fall back to knee ==
    // that probe's duty rather than dividing by an unset lo_i_mA.
    sweep_range_t r;
    sweep_range_begin(&r, PWM_RES, 20);

    bool has_panel = sweep_range_on_voc(&r, VOC_MV);
    TEST_ASSERT_TRUE(has_panel);

    uint32_t first_duty = r.duty;
    TEST_ASSERT_EQUAL_UINT32(SWEEP_PROBE_START_DUTY, first_duty);

    // Below the collapse threshold (15% of Voc) straight away.
    bool done = sweep_range_on_probe(&r, VOC_MV * 0.01f, 3.0f);

    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_EQUAL(SWEEP_RANGE_DONE, r.status);
    TEST_ASSERT_TRUE(r.collapsed);
    TEST_ASSERT_EQUAL_UINT32(first_duty, r.knee);
    TEST_ASSERT_TRUE(r.top >= r.min_top);
    TEST_ASSERT_TRUE(r.top <= r.hard_max);
}
