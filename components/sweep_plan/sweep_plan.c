#include "sweep_plan.h"

#include <math.h>

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
        // The collapsed reading is normally Isc, but a source that folds
        // back or cuts out (a bench supply in OCP, a flickering lamp) can
        // read far lower. Isc is never below the last regulating current,
        // so trust whichever is larger instead of shrinking the sweep.
        float isc_mA = (i_mA > r->lo_i_mA) ? i_mA : r->lo_i_mA;
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
            if (knee < r->lo)
                knee = r->lo;
            if (knee > r->hard_max)
                knee = r->hard_max;
        }
        sweep_range_finish(r, knee, isc_mA);
        // The collapsing probe's duty is known to be past Isc: make sure the
        // sweep can reach it even when the knee estimate is off (a reading
        // offset skews the mA-per-duty scale), so the curve always ends at
        // the collapse instead of stopping on the flat part.
        if (r->top < r->duty)
            r->top = (r->duty < r->hard_max) ? r->duty : r->hard_max;
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

// ── adaptive arc-length sweep ────────────────────────────────────────────

void sweep_adapt_begin(sweep_adapt_t *a, float voc_mV, float isc_mA,
                        uint32_t knee_duty, uint32_t top_duty, size_t n_points)
{
    a->top = top_duty;
    a->voc_mV = voc_mV;
    a->isc_mA = isc_mA;
    a->collapse_mV = voc_mV * SWEEP_ADAPT_COLLAPSE_PERCENT / 100.0f;

    float gain0 = (knee_duty > 0) ? (isc_mA / (float)knee_duty)
                                   : (isc_mA / (float)(top_duty > 0 ? top_duty : 1));
    if (gain0 <= 0.0f)
        gain0 = 1e-3f;
    a->gain0 = gain0;

    a->n_points = n_points;
    a->points_done = 0;
    a->next_duty = 0;

    a->prev_duty = 0;
    a->prev_v_mV = 0.0f;
    a->prev_i_mA = 0.0f;
    a->have_prev = false;

    a->refine_count = 0;
    a->refine_active = false;
    a->discard_last = false;
    a->refine_step = 0;
    a->refine_end = 0;

    a->done = false;
}

bool sweep_adapt_last_discarded(const sweep_adapt_t *a)
{
    return a->discard_last;
}

uint32_t sweep_adapt_next(sweep_adapt_t *a, float v_mV, float i_mA)
{
    if (a->done)
        return SWEEP_ADAPT_DONE;

    uint32_t duty = a->next_duty;
    a->points_done++;
    a->discard_last = false;

    float ds = SWEEP_ADAPT_TARGET_ARC / (float)(a->n_points - 1);
    bool collapsed = duty > 0 && v_mV <= a->collapse_mV;

    // A panel with a sharp knee can fall from Vmp to ~0 V within a couple of
    // duty steps, so a step can jump most of the way over the knee. If the
    // step just taken is much longer than the target arc and there is duty
    // room below it, drop this point and re-sweep that gap in small fixed
    // steps (from duty 0 again, since duty only ever steps up).
    if (!a->refine_active && a->refine_count < SWEEP_ADAPT_REFINE_MAX && a->have_prev &&
        duty >= a->prev_duty + 2 && a->n_points - a->points_done >= 2)
    {
        float gdv = (a->prev_v_mV - v_mV) / a->voc_mV;
        float gdi = (i_mA - a->prev_i_mA) / a->isc_mA;
        if (sqrtf(gdv * gdv + gdi * gdi) > SWEEP_ADAPT_REFINE_GAP * ds)
        {
            a->refine_count++;
            a->refine_active = true;
            a->discard_last = true;
            a->points_done--;

            size_t slots = a->n_points - a->points_done - 1; // keep one for the gap's end
            if (slots > SWEEP_ADAPT_REFINE_POINTS)
                slots = SWEEP_ADAPT_REFINE_POINTS;
            // Ceil, so the re-sweep reaches refine_end within its slots and
            // the rest of the budget still gets the curve to the collapse.
            uint32_t span = duty - a->prev_duty;
            uint32_t step = (span + (uint32_t)slots) / (uint32_t)(slots + 1);
            a->refine_step = step > 0 ? step : 1;
            a->refine_end = duty;

            uint32_t next = a->prev_duty + a->refine_step;
            if (next > duty)
                next = duty;
            a->next_duty = next;
            return next;
        }
    }
    if (a->refine_active && duty >= a->refine_end)
        a->refine_active = false;

    // Termination: collapsed (Isc reached, duty > 0 so the Voc point
    // itself never counts), duty cap reached, or point budget exhausted.
    if (collapsed || duty >= a->top || a->points_done >= a->n_points)
    {
        a->done = true;
        return SWEEP_ADAPT_DONE;
    }

    // Estimate the local normalized slope (dv/di) and the local measured
    // current gain (di/dduty) from the last two measured points. A dead
    // zone (di ~ 0 despite duty increasing) can't supply a usable gain,
    // so fall back to the auto-range scale and grow the step instead of
    // stalling.
    float slope = 0.0f;
    float gain = a->gain0;
    bool stalled = !a->have_prev;

    if (a->have_prev)
    {
        float dv = a->prev_v_mV - v_mV;
        float di = i_mA - a->prev_i_mA;
        float dduty = (float)duty - (float)a->prev_duty;

        float di_eps = a->isc_mA * 0.01f;
        if (di_eps < 0.05f)
            di_eps = 0.05f;

        if (di > di_eps && dduty > 0.0f)
        {
            gain = di / dduty;
            float norm_dv = dv / a->voc_mV;
            float norm_di = di / a->isc_mA;
            if (norm_di > 1e-6f)
                slope = norm_dv / norm_di;
        }
        else
        {
            stalled = true;
        }
    }

    float delta_i = ds / sqrtf(1.0f + slope * slope);
    float delta_duty = (delta_i * a->isc_mA) / gain;
    if (stalled)
        delta_duty *= SWEEP_ADAPT_DEADZONE_GROW;
    if (delta_duty < SWEEP_ADAPT_MIN_DUTY_STEP)
        delta_duty = SWEEP_ADAPT_MIN_DUTY_STEP;

    // Budget-aware clamp: leave headroom for remaining points when far
    // from the end, but force a jump toward top once the point budget
    // gets small, so the sweep always reaches Isc/top within n points.
    size_t remaining_slots = a->n_points - a->points_done; // slots left, including the next one
    uint32_t span = a->top - duty;

    if (remaining_slots <= SWEEP_ADAPT_JUMP_BUDGET_LEFT)
    {
        uint32_t forced = (span + (uint32_t)remaining_slots - 1) / (uint32_t)remaining_slots;
        if ((uint32_t)delta_duty < forced)
            delta_duty = (float)forced;
    }
    else if (delta_duty > (float)span)
    {
        // Never past top. (Halving the remaining span here instead made the
        // last points crawl toward top one duty step at a time on hardware.)
        delta_duty = (float)(span > 0 ? span : 1);
    }

    uint32_t next_duty = duty + (uint32_t)delta_duty;
    if (a->refine_active)
    {
        // Re-sweeping the knee: fixed small steps up to the known collapse.
        next_duty = duty + a->refine_step;
        if (next_duty > a->refine_end)
            next_duty = a->refine_end;
    }
    if (next_duty <= duty)
        next_duty = duty + 1;
    if (next_duty > a->top)
        next_duty = a->top;

    a->prev_duty = duty;
    a->prev_v_mV = v_mV;
    a->prev_i_mA = i_mA;
    a->have_prev = true;

    a->next_duty = next_duty;
    return next_duty;
}
