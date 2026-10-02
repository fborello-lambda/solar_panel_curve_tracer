#include "ui_chart.h"

#include <stdio.h>

#include "chart_math.h"
#include "driver_sh1106.h"

// Plot box: axes at x=PX0-2 / y=PY0+2, data area PX0..PX1 by PY1..PY0.
// PY1/PX1 leave 2 px for the 5x5 cursor box, so the highest point (the MPP
// on the power curve, Isc on the I-V curve) and Voc are not clipped.
#define PX0 5
#define PX1 124
#define PY0 48 // bottom (zero) row of the data area
#define PY1 3  // top row
#define PW (PX1 - PX0 + 1)
#define PH (PY0 - PY1 + 1)
#define FOOTER_Y 56
#define DASH_PX 2

static int px_of(float v, float vmax)
{
    return PX0 + chart_scale(v, vmax, PW);
}

static int py_of(float y, float ymax)
{
    return PY0 - chart_scale(y, ymax, PH);
}

void ui_chart_render(uint8_t *fb, const float *v, const float *i, int n, int sel, bool power_mode)
{
    // Axes: left edge and bottom edge, with a tick at each end.
    sh1106_fb_draw_line(fb, PX0 - 2, PY1 - 3, PX0 - 2, PY0 + 2, 0, true);
    sh1106_fb_draw_line(fb, PX0 - 2, PY0 + 2, PX1 + 3, PY0 + 2, 0, true);
    sh1106_fb_draw_line(fb, PX1, PY0 + 2, PX1, PY0 + 4, 0, true);

    if (n <= 0)
    {
        sh1106_fb_draw_text(fb, 0, FOOTER_Y, "NO DATA");
        return;
    }
    if (n > UI_CHART_MAX_POINTS)
    {
        n = UI_CHART_MAX_POINTS;
    }
    if (sel < 0)
    {
        sel = 0;
    }
    if (sel >= n)
    {
        sel = n - 1;
    }

    float vmax = 0.0f, imax = 0.0f, pmax = 0.0f;
    for (int k = 0; k < n; k++)
    {
        if (v[k] > vmax)
        {
            vmax = v[k];
        }
        if (i[k] > imax)
        {
            imax = i[k];
        }
        if (v[k] * i[k] > pmax)
        {
            pmax = v[k] * i[k];
        }
    }

    int sx = 0, sy = 0;
    for (int k = 0; k < n; k++)
    {
        int x = px_of(v[k], vmax);
        int y = py_of(i[k], imax);
        if (k > 0)
        {
            sh1106_fb_draw_line(fb, px_of(v[k - 1], vmax), py_of(i[k - 1], imax), x, y, 0, true);
        }
        sh1106_fb_draw_rect(fb, x - 1, y - 1, 3, 3, true, true);
    }

    if (power_mode)
    {
        for (int k = 1; k < n; k++)
        {
            sh1106_fb_draw_line(fb, px_of(v[k - 1], vmax), py_of(v[k - 1] * i[k - 1], pmax),
                                px_of(v[k], vmax), py_of(v[k] * i[k], pmax), DASH_PX, true);
        }
    }

    sx = px_of(v[sel], vmax);
    sy = power_mode ? py_of(v[sel] * i[sel], pmax) : py_of(i[sel], imax);

    // Dotted vertical guide through the selected point, then a hollow 5x5 box.
    for (int y = PY1 - 2; y <= PY0 + 1; y += 2)
    {
        sh1106_fb_set_pixel(fb, sx, y, true);
    }
    sh1106_fb_draw_rect(fb, sx - 2, sy - 2, 5, 5, true, false);
    sh1106_fb_draw_rect(fb, sx - 2, sy - 2, 5, 5, false, true);

    char foot[24];
    if (power_mode)
    {
        snprintf(foot, sizeof(foot), "%d/%d %.1fV %.0fmW", sel + 1, n, (double)v[sel], (double)(v[sel] * i[sel]));
    }
    else
    {
        snprintf(foot, sizeof(foot), "%d/%d %.1fV %.1fmA", sel + 1, n, (double)v[sel], (double)i[sel]);
    }
    sh1106_fb_draw_text(fb, 0, FOOTER_Y, foot);
}
