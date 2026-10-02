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

typedef enum
{
    MEASUREMENT_FAULT_NONE = 0,
    MEASUREMENT_FAULT_NO_PANEL, // open-circuit voltage below SWEEP_VOC_MIN_MV
    MEASUREMENT_FAULT_NO_LOAD,  // commanding more load did not raise the current
    MEASUREMENT_FAULT_REVERSED, // current flowing at ~0 V: panel reversed or shorted
} measurement_fault_t;

/** @brief Fault from the last REAL sweep (cleared when a new one starts). */
measurement_fault_t measurement_last_fault(void);
/** @brief "none", "no_panel", "no_load" or "reversed", for the /status JSON. */
const char *measurement_fault_str(measurement_fault_t fault);

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

/** @brief INA219 fixed zero offset in mA (subtract it from raw readings). */
float measurement_ina_offset_mA(void);

/**
 * @brief Re-measure the INA219 fixed zero offset now. The panel must be
 * disconnected (input below 0.5 V); returns false otherwise or when busy.
 */
bool measurement_capture_zero(void);

/**
 * @brief One-time bench calibration of the INA219 voltage-proportional error:
 * with a stiff source of a few volts on the input and the load idle, pass the
 * current a multimeter in series reads (the load's real idle current). Stored
 * in NVS; from then on sweeps report that idle current instead of zeroing it.
 */
bool measurement_calibrate_k(float real_idle_mA);
float measurement_ina_k_mA_per_V(void);

void measurement_set_producer_mode(curve_producer_mode_t mode);
curve_producer_mode_t measurement_get_producer_mode(void);
const char *measurement_get_producer_mode_label(void);
