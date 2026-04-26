#include "app_hw.h"

#include <esp_log.h>

#include "app_state.h"

static const char *TAG = "APP_HW";

esp_err_t app_hw_ensure_i2c_bus_ready(void)
{
    if (g_app.i2c_bus != NULL)
    {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, // using external pullups on PCB
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &g_app.i2c_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ensure_i2c_bus_ready: failed to init bus: %s", esp_err_to_name(ret));
    }

    return ret;
}
