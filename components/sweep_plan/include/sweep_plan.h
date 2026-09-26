#pragma once

/*
 * Pure sweep-plan math: adaptive arc-length duty placement and the
 * auto-range decision state machine. No hardware, no FreeRTOS, no I2C —
 * this is what lets it build and run on the `linux` IDF target for host
 * unit tests.
 *
 * All I/O (measuring a point, logging, honoring a stop request, power
 * limits) stays in main/measurement.c, which drives these functions.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// ── auto-range tunables ─────────────────────────────────────────────────
#define SWEEP_PROBE_START_DUTY 8         // first duty the doubling search tries
#define SWEEP_COLLAPSE_PERCENT_OF_VOC 15 // panel counts as collapsed below this % of Voc
#define SWEEP_DUTY_MAX_PERCENT 20        // hard ceiling on commanded duty, % of pwm_res
#define SWEEP_KNEE_HEADROOM_PERCENT 115  // sweep top = knee * this / 100
#define SWEEP_VOC_MIN_MV 500             // below this, no panel worth sweeping

    // ── auto-range state machine ────────────────────────────────────────

    typedef enum
    {
        SWEEP_RANGE_PROBING = 0, // waiting on the next probe result
        SWEEP_RANGE_NO_PANEL,    // Voc too low, nothing to sweep
        SWEEP_RANGE_DONE,        // top/knee/isc_mA are final
    } sweep_range_status_t;

    typedef struct
    {
        uint32_t pwm_res;
        uint32_t hard_max;
        uint32_t min_top; // sweep top is never smaller than this (== point count)

        uint32_t duty;   // next duty to probe (valid while PROBING)
        uint32_t lo;      // last non-collapsed probe duty
        float lo_i_mA;    // current measured at `lo`
        float collapse_mV;

        sweep_range_status_t status;

        // Results, valid once status == SWEEP_RANGE_DONE.
        uint32_t top;
        uint32_t knee;
        float isc_mA;
        float voc_mV;
        bool collapsed; // true if the panel collapsed below threshold; false
                         // if the duty cap was hit first (curve will stop
                         // short of Isc)
    } sweep_range_t;

    /**
     * @brief Start an auto-range decision sequence.
     *
     * @param pwm_res PWM counter resolution (steps).
     * @param min_top Sweep top is never smaller than this (the number of
     *                points the caller intends to place, e.g. DB_MAX_SAMPLES).
     */
    void sweep_range_begin(sweep_range_t *r, uint32_t pwm_res, uint32_t min_top);

    /**
     * @brief Feed the Voc probe result (duty 0).
     *
     * @return true if there is a panel worth sweeping (r->duty now holds
     *         the first loaded probe duty, status == SWEEP_RANGE_PROBING);
     *         false if Voc was below SWEEP_VOC_MIN_MV
     *         (status == SWEEP_RANGE_NO_PANEL).
     */
    bool sweep_range_on_voc(sweep_range_t *r, float voc_mV);

    /**
     * @brief Feed a loaded probe result at the duty last returned via
     * r->duty.
     *
     * @return true if auto-range has finished (status == SWEEP_RANGE_DONE,
     *         top/knee/isc_mA are set); false if another probe is needed
     *         (r->duty now holds the next duty to try).
     */
    bool sweep_range_on_probe(sweep_range_t *r, float v_mV, float i_mA);

    // ── adaptive arc-length sweep ────────────────────────────────────────
    //
    // Places points by normalized arc length along the curve (v = V/Voc,
    // i = I/Isc) instead of fixed fractions of the knee duty, so the steep
    // part near Voc, the knee, and the flat part near Isc all get points
    // for any Isc. Duty only ever steps up (the RC input filter drains
    // slowly, so a descending step would read stale current).

#define SWEEP_ADAPT_TARGET_ARC 2.0f     // total normalized arc length budget (S)
#define SWEEP_ADAPT_COLLAPSE_PERCENT 3  // done once V < this % of Voc
#define SWEEP_ADAPT_JUMP_BUDGET_LEFT 2  // when this few points remain, jump toward top
#define SWEEP_ADAPT_MIN_DUTY_STEP 1     // duty step floor
#define SWEEP_ADAPT_DEADZONE_GROW 2.0f  // step multiplier when current gain looks stalled

#define SWEEP_ADAPT_DONE UINT32_MAX // sentinel returned by sweep_adapt_next() when finished

    typedef struct
    {
        uint32_t top;      // sweep ceiling duty
        float voc_mV;
        float isc_mA;
        float collapse_mV; // V below this counts as collapsed (Isc reached)
        float gain0;        // fallback mA/duty gain, from auto-range's isc/knee

        size_t n_points;
        size_t points_done;

        uint32_t next_duty; // duty the point about to be fed was measured at

        uint32_t prev_duty;
        float prev_v_mV;
        float prev_i_mA;
        bool have_prev;

        bool done;
    } sweep_adapt_t;

    /**
     * @brief Start the adaptive sweep after auto-range has produced
     * voc_mV/isc_mA/knee_duty/top_duty.
     *
     * Feed the Voc probe itself (duty 0) as the first point via
     * sweep_adapt_next(a, voc_mV, ~0 mA) to get the first real step duty.
     */
    void sweep_adapt_begin(sweep_adapt_t *a, float voc_mV, float isc_mA,
                            uint32_t knee_duty, uint32_t top_duty, size_t n_points);

    /**
     * @brief Feed the point just measured (at the duty last returned, or
     * duty 0 for the first call) and get the next duty to command.
     *
     * @return the next duty to command (strictly greater than every duty
     *         fed so far), or SWEEP_ADAPT_DONE if the sweep is finished
     *         (V collapsed below the threshold, or duty/point budget is
     *         exhausted).
     */
    uint32_t sweep_adapt_next(sweep_adapt_t *a, float v_mV, float i_mA);

#ifdef __cplusplus
}
#endif
