#pragma once

#include <stdbool.h>
#include <stdint.h>

#define UI_CHART_MAX_POINTS 40

// Draws the I-V chart (v in V, i in mA) of n points into the 128x64 framebuffer,
// with the selected point `sel` highlighted and the footer text for it.
// power_mode overlays a dashed P(V) curve and shows power in the footer.
void ui_chart_render(uint8_t *fb, const float *v, const float *i, int n, int sel, bool power_mode);
