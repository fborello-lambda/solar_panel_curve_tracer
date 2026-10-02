#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Index of the point with the largest x*y (the maximum power point), 0 if n <= 0.
int chart_mpp_index(const float *x, const float *y, int n);

// Maps value in [0, vmax] to a pixel offset in [0, span_px - 1] (rounded,
// clamped). Returns 0 when vmax <= 0 or span_px <= 0.
int chart_scale(float value, float vmax, int span_px);

// True when a press at now_ms follows a press at prev_ms within window_ms.
// Unsigned subtraction, so it survives timestamp wraparound.
bool chart_is_double_press(uint32_t prev_ms, uint32_t now_ms, uint32_t window_ms);

#ifdef __cplusplus
}
#endif
