#include "json_builder.h"

#include <string.h>

size_t build_x_y_samples_json(char *buf, size_t cap,
                              const float *x_arr, const float *y_arr,
                              int max_samples, int count)
{
    if (!buf || cap == 0 || count <= 0)
        return 0;

    if (count > max_samples)
        count = max_samples;

    // Not even "[]" + NUL fits: leave the caller a valid empty string
    // rather than any attempt at a JSON array.
    if (cap < 3)
    {
        buf[0] = '\0';
        return 0;
    }

    size_t pos = 0;
    buf[pos++] = '[';

    int wrote = 0;
    for (int i = 0; i < count; ++i)
    {
        char elem[48];
        int n = snprintf(elem, sizeof(elem), "%s{\"x\":%.3f,\"y\":%.3f}",
                          (wrote ? "," : ""), x_arr[i], y_arr[i]);
        if (n < 0)
            break;

        // Only commit this element if it (plus the closing ']' and NUL)
        // fits; otherwise stop here so the output never contains a
        // truncated, invalid element.
        if (pos + (size_t)n + 2 > cap)
            break;

        memcpy(buf + pos, elem, (size_t)n);
        pos += (size_t)n;
        wrote++;
    }

    buf[pos++] = ']';
    buf[pos] = '\0';
    return pos;
}
