/*
 * In memory database for samples. It's a way to keep track of the state
 * of the system and share it across tasks.
 *
 * It also contains standalone variables to keep track of the state of the system.
 * These variables are accessed via getter/setter functions.
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

// Taken from the notebook analysis
// It depends on the resistor, the INA219 calibration, the power supply, etc.
// Also, the shunt resistor will determine the max current that can be measured due to power dissipation.
#define MAX_CURRENT_MA 3922.0f
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

    /**
     * @brief Get the current current setpoint in mA.
     *
     * It's the range of current wanted to get from the Solar Panel.
     *
     * @return The current setpoint in mA.
     */
    float db_get_current_setpoint_mA(void);

    /**
     * @brief Set the current current setpoint in mA.
     *
     * It's the range of current wanted to get from the Solar Panel.
     *
     * @param value The new current setpoint in mA. Clamped to [0, MAX_CURRENT_MA].
     */
    void db_set_current_setpoint_mA(float value);

#ifdef __cplusplus
}
#endif

#endif // MAIN_DB_H
