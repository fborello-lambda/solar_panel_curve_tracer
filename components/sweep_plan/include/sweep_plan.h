#pragma once

/*
 * Pure sweep-plan math: per-step duty placement and the auto-range
 * decision state machine. No hardware, no FreeRTOS, no I2C — this is what
 * lets it build and run on the `linux` IDF target for host unit tests.
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

// ── sweep point placement tunables ──────────────────────────────────────
#define SWEEP_FINE_LEG_POINTS 12         // points spent across the knee band
#define SWEEP_TAIL_LEG_POINTS 2          // points spent from the knee band to top
#define SWEEP_FINE_BAND_START_PERCENT 85 // fine band start, % of knee duty
#define SWEEP_FINE_BAND_END_PERCENT 102  // fine band end, % of knee duty

// ── auto-range tunables ─────────────────────────────────────────────────
#define SWEEP_PROBE_START_DUTY 8         // first duty the doubling search tries
#define SWEEP_COLLAPSE_PERCENT_OF_VOC 15 // panel counts as collapsed below this % of Voc
#define SWEEP_DUTY_MAX_PERCENT 20        // hard ceiling on commanded duty, % of pwm_res
#define SWEEP_KNEE_HEADROOM_PERCENT 115  // sweep top = knee * this / 100
#define SWEEP_VOC_MIN_MV 500             // below this, no panel worth sweeping

    /**
     * @brief Commanded duty for sweep step `step` of `n`, given the
     * auto-ranged `top` and the `knee` duty it was derived from.
     *
     * Three legs, not a linear ramp: a coarse leg over the flat below-knee
     * region (a panel there is a current source, so coarse steps lose
     * little), most of the budget across the knee band where the curve
     * actually bends, and a short tail up to `top` to pin Isc. This is the
     * raw per-leg placement only; it is not guaranteed strictly increasing
     * on its own (see sweep_plan_build()).
     */
    uint32_t sweep_duty_for_step(int step, uint32_t top, uint32_t knee, size_t n);

    /**
     * @brief Build `n` strictly-increasing sweep duties into `duties`.
     *
     * duties[0] == 0, duties[n-1] == top, every duty <= top, and each duty
     * is strictly greater than the previous one (sweep_duty_for_step's raw
     * placement is nudged up where needed, and clamped so every remaining
     * step still has room to strictly increase up to `top`).
     */
    void sweep_plan_build(uint32_t top, uint32_t knee, uint32_t *duties, size_t n);

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

#ifdef __cplusplus
}
#endif
