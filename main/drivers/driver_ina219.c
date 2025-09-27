#include "driver_ina219.h"

static const char *TAG = "ina219_driver";

#define TIMEOUT 1000

int ina219_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, i2c_port_num_t port, int sda_pin, int scl_pin, uint32_t clk_speed_hz, uint8_t i2c_addr)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = clk_speed_hz,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));

    return ESP_OK;
}

int ina219_reset(i2c_master_dev_handle_t dev_handle)
{
    // Write to the configuration register to reset the device
    // The reset bit is bit 15 of the configuration register
    return ina219_write_register(dev_handle, INA219_REG_CONFIG, 1 << 15);
}

/* Write arbitrary bytes to the device */
int ina219_write_bytes(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, const uint8_t *data, size_t len)
{
    if (!data || len == 0)
        return ESP_ERR_INVALID_ARG;

    uint8_t write_buf[len + 1];
    write_buf[0] = reg_addr;
    if (len)
        memcpy(&write_buf[1], data, len);

    return i2c_master_transmit(dev_handle, write_buf, (size_t)(len + 1), TIMEOUT);
}

/* Read arbitrary bytes from the device (starts a read transaction) */
int ina219_read_bytes(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (!data || len == 0)
        return ESP_ERR_INVALID_ARG;

    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, TIMEOUT);
}

int ina219_read_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    int ret = ina219_read_bytes(dev_handle, reg, data, sizeof(data));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(ret));
        return ret;
    }
    *value = (data[0] << 8) | data[1];
    return ESP_OK;
}

int ina219_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint16_t value)
{
    uint8_t data[2] = {value >> 8, value & 0xFF};
    int ret = ina219_write_bytes(dev_handle, reg, data, sizeof(data));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

int ina219_calibrate_for_32V_10A(i2c_master_dev_handle_t dev_handle, ina219_cal_t *cal)
{
    if (cal == NULL)
        return ESP_ERR_INVALID_ARG;
    if (dev_handle == NULL)
        return ESP_ERR_INVALID_ARG;

    const double Rshunt = 0.010; /* 10 mOhm */
    // const double max_expected_current = 10.0; /* 10 A */

    // const double current_lsb_1 = max_expected_current / (1 << 15); // (2^15)
    // const double current_lsb_2 = max_expected_current / (1 << 12); // (2^12)
    // const double current_lsb = (current_lsb_2 - current_lsb_1) / 2.0;
    const double current_lsb = 0.0005; /* 500 uA per bit */

    // Calibration value
    double cal_d = 0.04096 / (current_lsb * Rshunt);
    uint16_t cal_value = (uint16_t)(cal_d + 0.5);
    /* Power LSB = 20 * CurrentLSB (W/bit) */
    const double power_lsb = 20.0 * current_lsb;

    /* Fill cal struct (convenience integer multipliers) */
    cal->cal_value = cal_value;
    cal->current_lsb = current_lsb;
    /* current_divider_mA: mA = raw_current / current_divider_mA
       current_divider_mA = 1 / (current_lsb * 1000) = 1 / (0.0001 * 1000) = 10 */
    double tmp = 1.0 / (current_lsb * 1000.0);
    double plus = tmp + 0.5;
    ESP_LOGI(TAG, "current_divider=%f || plus=%f", tmp, plus);
    cal->current_divider_mA = (int)(1.0 / (current_lsb * 1000.0) + 0.5);
    /* power_multiplier_mW: mW = raw_power * power_multiplier_mW
       power_lsb in mW = power_lsb * 1000 = 2 -> multiplier = 2 */
    cal->power_multiplier_mW = (int)(power_lsb * 1000.0 + 0.5);
    cal->shunt_resistor_mOhm = 10; /* 10 mOhm */

    ESP_LOGI(TAG, "INA219 32V/10A calibration: cal=0x%04X, CurrentLSB=%.6f A/bit, PowerLSB=%.3f mW/bit",
             cal_value, current_lsb, power_lsb * 1000.0);
    ESP_LOGI(TAG, "Expected ranges: I_max(LSB)=%.2f A, Vshunt(10A)=%.0f mV, PGA=160 mV",
             current_lsb * 32767.0, 10.0 * Rshunt * 1000.0);

    /* Write calibration register */
    int ret = ina219_write_register(dev_handle, INA219_REG_CALIBRATION, cal_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write calibration register: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Config: 32V range, gain 4 (160mV), 12-bit ADC, continuous shunt+bus */
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V | INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US | INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    ret = ina219_write_register(dev_handle, INA219_REG_CONFIG, config);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to write config register: %s", esp_err_to_name(ret));
        /* calibration already written — proceed */
    }

    return ESP_OK;
}

int ina219_calibrate_for_32V_2A(i2c_master_dev_handle_t dev_handle, ina219_cal_t *cal)
{
    if (cal == NULL)
        return ESP_ERR_INVALID_ARG;
    if (dev_handle == NULL)
        return ESP_ERR_INVALID_ARG;

    /* Adafruit defaults for 32V, 2A, Rshunt = 0.1 ohm */
    const double Rshunt = 0.100; /* 100 mOhm -> 0.1 ohm */
    // const double max_expected_current = 2.0; /* 2 A */

    /* 1) Determine minimum current LSB */
    // const double min_current_lsb = max_expected_current / 32767.0;

    /* 2) Choose a nice round Current_LSB (Adafruit uses 100uA = 0.0001 A/bit) */
    const double current_lsb = 0.0001; /* 100 uA per bit */

    /* 3) Compute calibration register */
    double cal_double = 0.04096 / (current_lsb * Rshunt);
    uint16_t cal_value = (uint16_t)(cal_double + 0.5); /* round */

    /* 4) Power LSB (W/bit) */
    const double power_lsb = 20.0 * current_lsb; /* 0.002 W/bit */

    /* Fill cal struct (convenience integer multipliers) */
    cal->cal_value = cal_value;
    cal->current_lsb = current_lsb;
    /* current_divider_mA: mA = raw_current / current_divider_mA
       current_divider_mA = 1 / (current_lsb * 1000) = 1 / (0.0001 * 1000) = 10 */
    cal->current_divider_mA = (int)(1.0 / (current_lsb * 1000.0) + 0.5);
    /* power_multiplier_mW: mW = raw_power * power_multiplier_mW
       power_lsb in mW = power_lsb * 1000 = 2 -> multiplier = 2 */
    cal->power_multiplier_mW = (int)(power_lsb * 1000.0 + 0.5);
    cal->shunt_resistor_mOhm = 100; /* 100 mOhm */

    ESP_LOGI(TAG, "INA219 32V/2A calibration: cal=0x%04X, current_lsb=%g A/bit, power_lsb=%g W/bit",
             cal_value, current_lsb, power_lsb);
    ESP_LOGI(TAG, "convenience: current_divider_mA=%d, power_multiplier_mW=%d",
             cal->current_divider_mA, cal->power_multiplier_mW);

    /* Write calibration register */
    int ret = ina219_write_register(dev_handle, INA219_REG_CALIBRATION, cal_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write calibration register: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set config register matching Adafruit example:
       32V range, gain 8 (320mV), 12-bit ADC, continuous shunt+bus */
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V | INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US | INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    ret = ina219_write_register(dev_handle, INA219_REG_CONFIG, config);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to write config register: %s", esp_err_to_name(ret));
        /* still return success because calibration was written */
    }

    return ESP_OK;
}

/* Read shunt voltage (milivolts) */
int ina219_get_shunt_voltage_mv(i2c_master_dev_handle_t dev_handle, int32_t *mV)
{
    if (mV == NULL)
        return ESP_ERR_INVALID_ARG;
    uint16_t raw;
    int ret = ina219_read_register(dev_handle, INA219_REG_SHUNTVOLTAGE, &raw);
    if (ret != ESP_OK)
        return ret;
    int16_t s = (int16_t)raw;
    *mV = (int32_t)s * 0.01;
    return ESP_OK;
}

/* Read shunt voltage (microvolts) */
int ina219_get_shunt_voltage_uv(i2c_master_dev_handle_t dev_handle, int32_t *uV)
{
    if (uV == NULL)
        return ESP_ERR_INVALID_ARG;
    uint16_t raw;
    int ret = ina219_read_register(dev_handle, INA219_REG_SHUNTVOLTAGE, &raw);
    if (ret != ESP_OK)
        return ret;
    int16_t s = (int16_t)raw;
    *uV = (int32_t)s * 10;
    return ESP_OK;
}

/* Read bus voltage (millivolts) */
int ina219_get_bus_voltage_mv(i2c_master_dev_handle_t dev_handle, int32_t *mV)
{
    if (mV == NULL)
        return ESP_ERR_INVALID_ARG;
    uint16_t raw;
    int ret = ina219_read_register(dev_handle, INA219_REG_BUSVOLTAGE, &raw);
    if (ret != ESP_OK)
        return ret;
    /* Bits [15:3] contain voltage, LSB = 4mV */
    uint16_t v = (raw >> 3);
    *mV = (int32_t)v * 4;
    return ESP_OK;
}

/* Read current (milliamps) using calibration */
int ina219_get_current_ma(i2c_master_dev_handle_t dev_handle, const ina219_cal_t *cal, int32_t *mA)
{
    if (cal == NULL || mA == NULL)
        return ESP_ERR_INVALID_ARG;
    if (cal->current_divider_mA == 0)
        return ESP_ERR_INVALID_STATE;

    uint16_t raw;
    int ret = ina219_read_register(dev_handle, INA219_REG_CURRENT, &raw);
    if (ret != ESP_OK)
        return ret;
    int16_t s = (int16_t)raw;
    /* mA = raw / divider */
    *mA = (int32_t)s / cal->current_divider_mA;
    return ESP_OK;
}

/* Read power (milliwatts) using calibration */
int ina219_get_power_mw(i2c_master_dev_handle_t dev_handle, const ina219_cal_t *cal, int32_t *mW)
{
    if (cal == NULL || mW == NULL)
        return ESP_ERR_INVALID_ARG;

    uint16_t raw;
    int ret = ina219_read_register(dev_handle, INA219_REG_POWER, &raw);
    if (ret != ESP_OK)
        return ret;
    /* mW = raw * power_multiplier_mW */
    *mW = (int32_t)raw * cal->power_multiplier_mW;
    return ESP_OK;
}
