#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CURVE_PRODUCER_REAL = 0,
    CURVE_PRODUCER_DUMMY,
} curve_producer_mode_t;

typedef enum
{
    MEASUREMENT_REFUSE_NONE = 0,     // request succeeded, or stop while not running (no-op)
    MEASUREMENT_REFUSE_ALREADY_RUNNING,
    MEASUREMENT_REFUSE_DYNAMIC_LOAD_ACTIVE,
    MEASUREMENT_REFUSE_SENSOR_NOT_READY,
} measurement_refuse_reason_t;

bool measurement_is_running(void);
bool measurement_request(bool start);

/**
 * @brief Same as measurement_request(), but also reports why a start was
 * refused via `out_reason` (ignored on stop, or may be NULL).
 */
bool measurement_request_ex(bool start, measurement_refuse_reason_t *out_reason);

/**
 * @brief Human-readable string for a measurement_refuse_reason_t, suitable
 * for a JSON error message.
 */
const char *measurement_refuse_reason_str(measurement_refuse_reason_t reason);

bool measurement_init_load_control_hw(void);

/**
 * @brief Debug: step the load from duty 0 to max_duty in fixed steps and log
 * one raw CSV line per step (no calibration, no db). Refuses while a sweep
 * or dynamic load is active.
 */
bool measurement_raw_scan(uint32_t max_duty, uint32_t step);

/**
 * @brief Averaged INA219 reading over the sweep's 100 ms window at the
 * current load, without the zero-load correction: bus voltage in mV and
 * raw shunt current in mA. Used by the dynamic load screen.
 */
bool measurement_sample(float *out_bus_mV, float *out_raw_mA);

void measurement_set_producer_mode(curve_producer_mode_t mode);
curve_producer_mode_t measurement_get_producer_mode(void);
const char *measurement_get_producer_mode_label(void);
