#include <math.h>
#include <stdint.h>

#include "unity.h"
#include "sweep_plan.h"

#define PWM_RES 8191u
#define SCALE_MA_PER_DUTY 0.479f // synthetic panel: mA per duty step at low duty
#define VOC_MV 20000.0f          // synthetic panel Voc, 20 V

// Intentionally hardcoded here rather than reusing
// SWEEP_KNEE_HEADROOM_PERCENT from sweep_plan.h: this value backs an
// independent re-derivation of r.top from r.knee, so a mutation to the
// production macro's value changes r.top but not this expectation, and
// the test actually catches it (reusing the macro would mutate both sides
// together and the test would still pass).
#define EXPECTED_KNEE_HEADROOM_PERCENT 115

// Re-derives the expected top duty from a finished sweep_range_t the same
// way sweep_range_finish() does, but independently of its implementation
// (see EXPECTED_KNEE_HEADROOM_PERCENT above).
static uint32_t expected_top_from_knee(const sweep_range_t *r)
{
    uint32_t top = (uint32_t)(((uint64_t)r->knee * EXPECTED_KNEE_HEADROOM_PERCENT) / 100);
    if (top > r->hard_max)
        top = r->hard_max;
    if (top < r->min_top)
        top = r->min_top;
    return top;
}

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

        TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected_top_from_knee(&r), r.top,
                                          "top is not knee*115%, clamped to [min_top, hard_max]");
    }
}

void test_sweep_range_knee_times_headroom_clamped_to_hard_max(void)
{
    // panel_probe()'s logistic collapse model can't actually reach a knee
    // this close to hard_max (its steepness bounds the achievable knee well
    // under hard_max/1.15, see the knee-estimate test above), so this drives
    // sweep_range_on_probe() directly with hand-picked readings instead of
    // going through run_auto_range()'s simulated doubling search.
    sweep_range_t r;
    sweep_range_begin(&r, PWM_RES, 20);

    bool has_panel = sweep_range_on_voc(&r, VOC_MV);
    TEST_ASSERT_TRUE(has_panel);
    TEST_ASSERT_EQUAL_UINT32(SWEEP_PROBE_START_DUTY, r.duty); // == 8

    // First probe (duty 8): not collapsed. lo=8, lo_i_mA=3.832 (8*0.479).
    bool done = sweep_range_on_probe(&r, VOC_MV * 0.9f, 3.832f);
    TEST_ASSERT_FALSE(done);
    TEST_ASSERT_EQUAL_UINT32(16, r.duty); // doubled

    // Second probe (duty 16): collapsed, with i_mA picked so that
    // knee = i_mA * lo / lo_i_mA = 766.4 * 8 / 3.832 ~= 1600, which is
    // < hard_max (1638, so the on_probe knee>hard_max clamp doesn't fire)
    // but knee * 115 / 100 ~= 1840, which IS > hard_max: only
    // sweep_range_finish()'s clamp keeps r.top at hard_max.
    done = sweep_range_on_probe(&r, VOC_MV * 0.05f, 766.4f);
    TEST_ASSERT_TRUE(done);

    TEST_ASSERT_EQUAL(SWEEP_RANGE_DONE, r.status);
    TEST_ASSERT_TRUE(r.collapsed);
    TEST_ASSERT_TRUE_MESSAGE(r.knee < r.hard_max, "test setup invalid: knee already clamped to hard_max");
    TEST_ASSERT_TRUE_MESSAGE((uint64_t)r.knee * EXPECTED_KNEE_HEADROOM_PERCENT / 100 > r.hard_max,
                              "test setup invalid: knee*115% did not actually exceed hard_max");
    TEST_ASSERT_EQUAL_UINT32(r.hard_max, r.top);
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
