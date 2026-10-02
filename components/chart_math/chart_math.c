#include "chart_math.h"

int chart_mpp_index(const float *x, const float *y, int n)
{
    int best = 0;
    for (int i = 1; i < n; i++)
    {
        if (x[i] * y[i] > x[best] * y[best])
        {
            best = i;
        }
    }
    return best;
}

int chart_scale(float value, float vmax, int span_px)
{
    if (vmax <= 0.0f || span_px <= 0)
    {
        return 0;
    }
    if (!(value > 0.0f))
    {
        return 0; // also NaN and -inf, which must not reach the float to int cast
    }
    if (!(value < vmax))
    {
        return span_px - 1; // also +inf
    }
    int p = (int)(value / vmax * (float)(span_px - 1) + 0.5f);
    if (p < 0)
    {
        return 0;
    }
    if (p > span_px - 1)
    {
        return span_px - 1;
    }
    return p;
}

bool chart_is_double_press(uint32_t prev_ms, uint32_t now_ms, uint32_t window_ms)
{
    return (uint32_t)(now_ms - prev_ms) <= window_ms;
}
