#include "json_builder.h"

size_t build_x_y_samples_json(char *buf, size_t cap,
                              const float *x_arr, const float *y_arr,
                              int max_samples, int count)
{
    if (!buf || cap == 0 || count <= 0)
        return 0;

    if (count > max_samples)
        count = max_samples;

    size_t pos = 0;
    buf[pos++] = '[';

    int wrote = 0;
    for (int i = 0; i < count; ++i)
    {
        // conservative remaining-space check
        size_t rem = (pos < cap) ? (cap - pos) : 0;
        if (rem < 24)
        { // not enough room for another {"x":...,"y":...}
            pos = cap - 1;
            break;
        }

        int n = snprintf(buf + pos, rem, "%s{\"x\":%.3f,\"y\":%.3f}", (wrote ? "," : ""), x_arr[i], y_arr[i]);
        if (n < 0)
            break;
        if ((size_t)n >= rem)
        {
            pos = cap - 1;
            break;
        }
        pos += (size_t)n;
        wrote++;
    }

    if (pos < cap - 1)
    {
        buf[pos++] = ']';
        buf[pos] = '\0';
    }
    else
    {
        buf[cap - 2] = ']';
        buf[cap - 1] = '\0';
        pos = cap - 1;
    }
    return pos;
}
