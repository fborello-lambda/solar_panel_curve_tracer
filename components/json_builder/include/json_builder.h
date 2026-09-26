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
     * Contract: the output written to `buf` is always a complete, valid,
     * null-terminated JSON array containing only whole elements -- it is
     * never truncated mid-element. If not every sample fits in `cap` bytes,
     * as many leading elements as fit are emitted and the rest are simply
     * omitted (e.g. "[]" or "[{...},{...}]", never a cut-off "[{...",).
     *
     * @param buf Pointer to the buffer where the JSON string will be written.
     * @param cap The capacity of the buffer in bytes.
     * @param x_arr Pointer to the array of x values.
     * @param y_arr Pointer to the array of y values.
     * @param max_samples The maximum number of samples in the circular buffer.
     * @param count The number of valid samples currently in the buffer.
     * @return The number of bytes written to buf, excluding the terminating
     *         NUL (i.e. strlen(buf)). Returns 0 if buf is NULL, count <= 0,
     *         or cap is too small to hold even "[]" plus a NUL terminator
     *         (buf is left untouched, except a lone NUL at buf[0] if cap>0).
     */
    size_t build_x_y_samples_json(char *buf, size_t cap,
                                  const float *x_arr, const float *y_arr,
                                  int max_samples, int count);

#ifdef __cplusplus
}
#endif

#endif // JSON_BUILDER_H
