/*
 * Simple Way to Build JSON Strings
 */

#ifndef JSON_BUILDER_H
#define JSON_BUILDER_H

#include <stddef.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Build a JSON array of samples from the provided x and y arrays.
     * The JSON format is: [{x:...,y:...}, ...] from oldest to newest.
     *
     * @param buf Pointer to the buffer where the JSON string will be written.
     * @param cap The capacity of the buffer in bytes.
     * @param x_arr Pointer to the array of x values.
     * @param y_arr Pointer to the array of y values.
     * @param max_samples The maximum number of samples in the circular buffer.
     * @param count The number of valid samples currently in the buffer.
     * @return The number of bytes written to the buffer, or the required size if truncated.
     *         The buffer is always null-terminated if cap > 0.
     *         If the buffer is too small, the JSON will be truncated but still valid.
     *         If buf is NULL or cap is 0, returns 0.
     */
    size_t build_x_y_samples_json(char *buf, size_t cap,
                                  const float *x_arr, const float *y_arr,
                                  int max_samples, int count);

#ifdef __cplusplus
}
#endif

#endif // JSON_BUILDER_H
