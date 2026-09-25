#include "sweep_plan.h"

uint32_t sweep_duty_for_step(int step, uint32_t top, uint32_t knee, size_t n)
{
    const int coarse_points = (int)n - SWEEP_FINE_LEG_POINTS - SWEEP_TAIL_LEG_POINTS;
    uint32_t band_start = knee * SWEEP_FINE_BAND_START_PERCENT / 100;
    uint32_t band_end = knee * SWEEP_FINE_BAND_END_PERCENT / 100;
    if (band_end > top)
        band_end = top;

    if (step < coarse_points)
    {
        return (uint32_t)((uint64_t)step * band_start / coarse_points);
    }
    else if (step < coarse_points + SWEEP_FINE_LEG_POINTS)
    {
        uint32_t fine_step = (uint32_t)(step - coarse_points);
        uint32_t span = (band_end > band_start) ? (band_end - band_start) : 0;
        return band_start + (uint32_t)((uint64_t)fine_step * span / (SWEEP_FINE_LEG_POINTS - 1));
    }
    else
    {
        uint32_t tail_step = (uint32_t)(step - coarse_points - SWEEP_FINE_LEG_POINTS + 1);
        uint32_t span = (top > band_end) ? (top - band_end) : 0;
        return band_end + (uint32_t)((uint64_t)tail_step * span / SWEEP_TAIL_LEG_POINTS);
    }
}

void sweep_plan_build(uint32_t top, uint32_t knee, uint32_t *duties, size_t n)
{
    uint32_t prev_duty = 0;
    for (size_t step = 0; step < n; step++)
    {
        uint32_t duty = sweep_duty_for_step((int)step, top, knee, n);
        if (step > 0 && duty <= prev_duty)
            duty = prev_duty + 1;
        // Leave room for every remaining step to still strictly increase up
        // to `top`, so the bump above never crowds the final points.
        size_t steps_left = n - 1 - step;
        if (steps_left > 0 && duty > top - (uint32_t)steps_left)
            duty = top - (uint32_t)steps_left;
        if (step == n - 1)
            duty = top;
        prev_duty = duty;
        duties[step] = duty;
    }
}

void sweep_range_begin(sweep_range_t *r, uint32_t pwm_res, uint32_t min_top)
{
    uint32_t hard_max = pwm_res * SWEEP_DUTY_MAX_PERCENT / 100;
    if (hard_max < 1)
        hard_max = 1;

    r->pwm_res = pwm_res;
    r->hard_max = hard_max;
    r->min_top = min_top;
    r->duty = 0;
    r->lo = 0;
    r->lo_i_mA = 0.0f;
    r->collapse_mV = 0.0f;
    r->status = SWEEP_RANGE_PROBING;
    r->top = 0;
    r->knee = 0;
    r->isc_mA = 0.0f;
    r->voc_mV = 0.0f;
    r->collapsed = false;
}

bool sweep_range_on_voc(sweep_range_t *r, float voc_mV)
{
    r->voc_mV = voc_mV;

    if (voc_mV < SWEEP_VOC_MIN_MV)
    {
        r->status = SWEEP_RANGE_NO_PANEL;
        return false;
    }

    r->collapse_mV = voc_mV * SWEEP_COLLAPSE_PERCENT_OF_VOC / 100.0f;
    r->lo = 0;
    r->lo_i_mA = 0.0f;
    r->duty = (SWEEP_PROBE_START_DUTY < r->hard_max) ? SWEEP_PROBE_START_DUTY : r->hard_max;
    r->status = SWEEP_RANGE_PROBING;
    return true;
}

// Applies the knee-headroom rule shared by both the collapsed and the
// never-collapsed exit paths.
static void sweep_range_finish(sweep_range_t *r, uint32_t knee, float isc_mA)
{
    uint32_t top = (uint32_t)(((uint64_t)knee * SWEEP_KNEE_HEADROOM_PERCENT) / 100);
    if (top > r->hard_max)
        top = r->hard_max;
    if (top < r->min_top)
        top = r->min_top;

    r->top = top;
    r->knee = knee;
    r->isc_mA = isc_mA;
    r->collapsed = true;
    r->status = SWEEP_RANGE_DONE;
}

bool sweep_range_on_probe(sweep_range_t *r, float v_mV, float i_mA)
{
    if (v_mV <= r->collapse_mV)
    {
        float isc_mA = i_mA;
        uint32_t knee;
        if (r->lo == 0 || r->lo_i_mA <= 0.5f)
        {
            // Collapsed on the very first loaded probe: no regulating
            // point to scale from, so fall back to the collapsing duty.
            knee = r->duty;
        }
        else
        {
            knee = (uint32_t)((isc_mA * (float)r->lo) / r->lo_i_mA);
            if (knee > r->hard_max)
                knee = r->hard_max;
        }
        sweep_range_finish(r, knee, isc_mA);
        return true;
    }

    r->lo = r->duty;
    r->lo_i_mA = i_mA;

    if (r->duty >= r->hard_max)
    {
        // Never collapsed up to the duty cap: curve will stop short of Isc.
        r->top = r->hard_max;
        r->knee = r->hard_max * 100 / SWEEP_KNEE_HEADROOM_PERCENT;
        r->isc_mA = i_mA;
        r->collapsed = false;
        r->status = SWEEP_RANGE_DONE;
        return true;
    }

    r->duty = (r->duty * 2 < r->hard_max) ? r->duty * 2 : r->hard_max;
    return false;
}
