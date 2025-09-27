/*
 * driver_ina219.h
 *
 * Lightweight, portable header for INA219 current/power monitor
 *
 * This header declares the public API for an INA219 driver implementation.
 * It mirrors the common functions found in typical INA219 C implementations:
 * - initialization
 * - calibration presets
 * - read register helpers
 * - read shunt/bus voltage, current and power
 *
 * Datasheet: https://www.ti.com/lit/ds/symlink/ina219.pdf
 *
 * Adapted from ESP-IDF i2c_basic/main/i2c_basic_example_main.c:
 * https://github.com/espressif/esp-idf/blob/master/examples/peripherals/i2c/i2c_basic/main/i2c_basic_example_main.c
 *
 * Using as reference the Adafruit INA219 library:
 * https://github.com/adafruit/Adafruit_INA219
 */

#ifndef DRIVER_INA219_H
#define DRIVER_INA219_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2c_master.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Default I2C 7-bit address for INA219 */
#define INA219_ADDRESS_DEFAULT 0x40

/* INA219 register addresses */
#define INA219_REG_CONFIG 0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

/* Configuration bits (values taken from common INA219 defs / Adafruit) */
#define INA219_CONFIG_BVOLTAGERANGE_32V (0x2000)
#define INA219_CONFIG_BADCRES_12BIT (0x0180)
#define INA219_CONFIG_SADCRES_12BIT_1S_532US (0x0018)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)

#define INA219_CONFIG_GAIN_1_40MV (0x0000)
#define INA219_CONFIG_GAIN_2_80MV (0x0800)
#define INA219_CONFIG_GAIN_4_160MV (0x1000)
#define INA219_CONFIG_GAIN_8_320MV (0x1800)

    typedef struct
    {
        /* calibration register value written to INA219 */
        uint16_t cal_value;
        /* LSB of current in amperes (A/bit) */
        double current_lsb;
        /* convenience integer: divider to convert raw current to mA (mA = raw / current_divider_mA)
           (keeps API simple for integer math) */
        int current_divider_mA;
        /* convenience integer: multiplier to convert raw power to mW (mW = raw_power * power_multiplier_mW) */
        int power_multiplier_mW;
        int shunt_resistor_mOhm;
    } ina219_cal_t;

    /**
     * @brief Initialize an INA219 device and its I2C master resources.
     *
     * This function configures and installs the I2C driver for the given port and
     * pins (if bus_handle and dev_handle are provided they may be used/filled by
     * the implementation). It prepares the device handle for subsequent INA219
     * operations.
     *
     * @param bus_handle Pointer to an i2c_master_bus_handle_t that may be used or
     *                   filled by the implementation (may be NULL if not used).
     * @param dev_handle Pointer to an i2c_master_dev_handle_t that will be filled
     *                   with the INA219 device handle on success.
     * @param port I2C port number to use for the INA219.
     * @param sda_pin GPIO number used for SDA.
     * @param scl_pin GPIO number used for SCL.
     * @param clk_speed_hz I2C clock speed in Hz.
     * @param i2c_addr 7-bit I2C address of the INA219 device.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     *
     * Example usage:
     * ```c
     *  i2c_master_bus_handle_t bus_handle;
     *  i2c_master_dev_handle_t dev_handle;
     *  esp_err_t res = ina219_init(&bus_handle, &dev_handle, I2C_NUM_0, 8, 9, 100000, INA219_ADDRESS_DEFAULT);
     *  ina219_cal_t cal;
     *  ina219_calibrate_for_32V_10A(dev_handle, &cal);
     *
     *  int32_t shunt_uV = 0;
     *  ina219_get_shunt_voltage_uv(dev_handle, &shunt_uV);
     * ```
     */
    int ina219_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, i2c_port_num_t port, int sda_pin, int scl_pin, uint32_t clk_speed_hz, uint8_t i2c_addr);

    /**
     * @brief Reset the INA219 device to its default register values.
     *
     * This issues the appropriate write to the device reset field so the INA219
     * returns to its power-up defaults.
     *
     * @param dev_handle The I2C device handle for the INA219.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     */
    int ina219_reset(i2c_master_dev_handle_t dev_handle);

    /**
     * @brief Read a 16-bit register from the INA219.
     *
     * Reads the contents of the register specified by @p reg and returns the raw
     * 16-bit register value. The value is the device register as a uint16_t; the
     * caller is responsible for interpreting bit fields and signedness as documented
     * in the INA219 datasheet.
     *
     * @param dev_handle The I2C device handle for the INA219.
     * @param reg Register address to read.
     * @param value Pointer to uint16_t that will receive the register value on success.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     */
    // Should only be used for debugging or advanced use cases.
    int ina219_read_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint16_t *value);

    /**
     * @brief Write a 16-bit value to an INA219 register.
     *
     * Writes the 16-bit @p value to the device register specified by @p reg. The
     * value should be provided as the register's raw 16-bit representation.
     *
     * @param dev_handle The I2C device handle for the INA219.
     * @param reg Register address to write.
     * @param value 16-bit value to write to the register.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     */
    // Should only be used for debugging or advanced use cases.
    int ina219_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint16_t value);

    /**
     * @brief Calibrate the INA219 for a 32 V, 10 A maximum system with a 0.010 Ω shunt.
     *
     * This follows the Adafruit INA219 calibration approach:
     *  - Chooses a Current_LSB >= MaxExpected_I / 32767,
     *  - Computes the CALIBRATION register value and writes it to the device,
     *  - Computes and fills convenience conversion factors in the provided
     *    ina219_cal_t structure so raw current and power registers can be converted
     *    to milliamps and milliwatts respectively.
     *
     * It's specifically designed for a 0.010 Ω shunt resistor and a maximum
     * current of 10 A.
     *
     * @param dev_handle The I2C device handle for the INA219.
     * @param cal Pointer to an ina219_cal_t structure that will be filled with
     *            calibration and conversion parameters. Must not be NULL.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     */
    int ina219_calibrate_for_32V_10A(i2c_master_dev_handle_t dev_handle, ina219_cal_t *cal);

    /**
     * @brief Calibrate the INA219 for the default Adafruit configuration: 32 V, 2 A with 0.100 Ω shunt.
     *
     * Mirrors Adafruit's setCalibration_32V_2A:
     *  - VBUS_MAX = 32V
     *  - VSHUNT_MAX = 0.32V (Gain = 8)
     *  - RSHUNT = 0.100 Ω
     *  - Uses CurrentLSB = 100 µA/bit and computes CALIBRATION = 4096
     *
     * The function writes the calibration register and fills the provided ina219_cal_t
     * with conversion factors for current (mA) and power (mW).
     *
     * @param dev_handle The I2C device handle for the INA219.
     * @param cal Pointer to an ina219_cal_t structure that will be filled with
     *            calibration and conversion parameters. Must not be NULL.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     */
    int ina219_calibrate_for_32V_2A(i2c_master_dev_handle_t dev_handle, ina219_cal_t *cal);

    /**
     * @brief Read the shunt voltage and return the result in millivolts.
     *
     * The INA219 shunt voltage register LSB = 10 µV. This function converts the
     * signed raw register value to millivolts:
     *   mV = raw_shunt * 0.01
     *
     * @param dev_handle The I2C device handle for the INA219.
     * @param mV Pointer to int32_t that will receive the shunt voltage in millivolts.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     */
    int ina219_get_shunt_voltage_mv(i2c_master_dev_handle_t dev_handle, int32_t *mV);

    /**
     * @brief Read the shunt voltage and return the result in microvolts.
     *
     * The INA219 shunt voltage register LSB = 10 µV. This function converts the
     * signed raw register value to microvolts:
     *   uV = raw_shunt * 10
     *
     * @param dev_handle The I2C device handle for the INA219.
     * @param uV Pointer to int32_t that will receive the shunt voltage in microvolts.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     */
    int ina219_get_shunt_voltage_uv(i2c_master_dev_handle_t dev_handle, int32_t *uV);

    /**
     * @brief Read the bus voltage and return the result in millivolts.
     *
     * The INA219 bus voltage register contains voltage in bits [15:3] with LSB = 4 mV.
     * This function extracts the voltage field and converts it to millivolts:
     *   mV = (raw >> 3) * 4
     *
     * @param dev_handle The I2C device handle for the INA219.
     * @param mV Pointer to int32_t that will receive the bus voltage in millivolts.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     */
    int ina219_get_bus_voltage_mv(i2c_master_dev_handle_t dev_handle, int32_t *mV);

    /**
     * @brief Read the current register and convert to milliamps using calibration.
     *
     * Requires a prior successful call to one of the ina219_calibrate_* functions to
     * populate @p cal. The conversion is:
     *   current_mA = raw_current / cal->current_divider_mA
     *
     * @param dev_handle The I2C device handle for the INA219.
     * @param cal Pointer to a populated ina219_cal_t structure from calibration. Must not be NULL.
     * @param mA Pointer to int32_t that will receive the current in milliamps.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     */
    int ina219_get_current_ma(i2c_master_dev_handle_t dev_handle, const ina219_cal_t *cal, int32_t *mA);

    /**
     * @brief Read the power register and convert to milliwatts using calibration.
     *
     * Requires a prior successful call to one of the ina219_calibrate_* functions to
     * populate @p cal. The conversion is:
     *   power_mW = raw_power * cal->power_multiplier_mW
     *
     * @param dev_handle The I2C device handle for the INA219.
     * @param cal Pointer to a populated ina219_cal_t structure from calibration. Must not be NULL.
     * @param mW Pointer to int32_t that will receive the power in milliwatts.
     *
     * @return int 0 (ESP_OK) on success, or a non-zero esp_err_t error code on failure.
     */
    int ina219_get_power_mw(i2c_master_dev_handle_t dev_handle, const ina219_cal_t *cal, int32_t *mW);
#ifdef __cplusplus
}
#endif

#endif /* DRIVER_INA219_H */
