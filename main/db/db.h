/*
 * In memory database for samples. It's a way to keep track of the state
 * of the system and share it across tasks.
 *
 * Uses a mutex to protect access to the data.
 * Maybe not ideal for high-throughput, but sufficient for low-rate sampling.
 */

#ifndef MAIN_DB_H
#define MAIN_DB_H

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define DB_MAX_SAMPLES 20

    /**
     * @brief Initialize the database mutex/semaphore.
     */
    void db_init(void);

    /**
     * @brief Reset the database, just reset head and count; data will be overwritten.
     *
     * @return true on success, false on failure (e.g. could not obtain mutex)
     */
    bool db_reset(void);

    /**
     * @brief Add a new sample (x,y) to the database.
     * If the database is full, the oldest sample is overwritten.
     *
     * @param x The x value of the sample.
     * @param y The y value of the sample.
     *
     * @return true on success, false on failure (e.g. could not obtain mutex)
     */
    bool db_add(float x, float y);

    /**
     * @brief Snapshot the current samples in the database into the provided arrays.
     * Copies up to `cap` samples into `x_out` and `y_out` in oldest-to-newest order.
     *
     * @param x_out Pointer to an array where x values will be stored.
     * @param y_out Pointer to an array where y values will be stored.
     * @param cap The maximum number of samples to copy (size of x_out and y_out).
     * @param count Pointer to an integer where the actual number of samples copied will be stored.
     *
     * @return true on success, false on failure (e.g. invalid args or could not obtain mutex)
     */
    bool db_snapshot(float *x_out, float *y_out, size_t *count, size_t cap);

#ifdef __cplusplus
}
#endif

#endif // MAIN_DB_H
