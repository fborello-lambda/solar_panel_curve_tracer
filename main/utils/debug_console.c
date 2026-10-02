#include "debug_console.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <driver/usb_serial_jtag.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "measurement.h"
#include "dynamic_load.h"
#include "app/app_tasks.h"
#include "app/app_state.h"
#include "driver_ina219.h"
#include "pwm_controller.h"

static const char *TAG = "DEBUG";

// Line commands over the USB serial port, for bench debugging:
//   sweep            start a REAL sweep (same as START TRACE)
//   stop             stop a running sweep
//   scan <max> <st>  raw fixed-step load scan, one CSV line per step
//   gain <1|2|4|8>   INA219 shunt PGA (+-40/80/160/320 mV)
//   avg <1..128>     INA219 shunt ADC hardware averaging (samples)
//   raw <duty> <n>   hold duty, dump n single shunt register reads (10 uV units)
//   dyn on|off|up|down  drive the dynamic load screen (logs DYN,... lines)
//   zero             re-measure the INA219 fixed offset (panel disconnected)
//   kcal <mA>        calibrate the INA219 per-volt error; <mA> = idle current a
//                    multimeter in series reads with a supply on the input
static void ina_update_config(uint16_t mask, uint16_t bits)
{
    if (g_app.measurement_running || g_app.dynamic_load_active)
    {
        ESP_LOGW(TAG, "gain/avg: refused, a sweep or the dynamic load is using the INA219");
        return;
    }
    uint16_t cfg = 0;
    if (ina219_read_register(g_app.ina_dev, INA219_REG_CONFIG, &cfg) != ESP_OK)
    {
        ESP_LOGW(TAG, "config read failed");
        return;
    }
    cfg = (uint16_t)((cfg & ~mask) | bits);
    ina219_write_register(g_app.ina_dev, INA219_REG_CONFIG, cfg);
    ESP_LOGI(TAG, "INA219 config = 0x%04X", cfg);
}

static void raw_dump(uint32_t duty, int n)
{
    if (g_app.measurement_running || g_app.dynamic_load_active)
    {
        ESP_LOGW(TAG, "raw: busy");
        return;
    }
    pwm_controller_set_duty_in_res_steps(duty);
    vTaskDelay(pdMS_TO_TICKS(400));
    int16_t v[256];
    if (n > 256)
        n = 256;
    for (int k = 0; k < n; k++)
    {
        uint16_t r = 0;
        ina219_read_register(g_app.ina_dev, INA219_REG_SHUNTVOLTAGE, &r);
        v[k] = (int16_t)r;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    pwm_controller_set_duty_in_res_steps(0);
    // Print the distinct values and their counts: the gaps between them are
    // the real conversion step.
    printf("RAW duty=%lu n=%d values(x10uV):", (unsigned long)duty, n);
    for (int k = 0; k < n; k++)
    {
        bool seen = false;
        for (int j = 0; j < k; j++)
            if (v[j] == v[k])
                seen = true;
        if (seen)
            continue;
        int c = 0;
        for (int j = k; j < n; j++)
            if (v[j] == v[k])
                c++;
        printf(" %d:%d", v[k], c);
    }
    printf("\n");
}

static void handle_line(char *line)
{
    char *cmd = strtok(line, " \t\r\n");
    if (!cmd)
        return;

    if (strcmp(cmd, "sweep") == 0)
    {
        measurement_set_producer_mode(CURVE_PRODUCER_REAL);
        measurement_refuse_reason_t reason = MEASUREMENT_REFUSE_NONE;
        if (!measurement_request_ex(true, &reason))
            ESP_LOGW(TAG, "sweep refused: %s", measurement_refuse_reason_str(reason));
    }
    else if (strcmp(cmd, "stop") == 0)
    {
        measurement_request(false);
    }
    else if (strcmp(cmd, "scan") == 0)
    {
        char *a = strtok(NULL, " \t\r\n");
        char *b = strtok(NULL, " \t\r\n");
        uint32_t max_duty = a ? (uint32_t)strtoul(a, NULL, 10) : 400;
        uint32_t step = b ? (uint32_t)strtoul(b, NULL, 10) : 10;
        measurement_raw_scan(max_duty, step);
    }
    else if (strcmp(cmd, "gain") == 0)
    {
        char *a = strtok(NULL, " \t\r\n");
        int g = a ? atoi(a) : 4;
        uint16_t bits = (g == 1) ? INA219_CONFIG_GAIN_1_40MV : (g == 2) ? INA219_CONFIG_GAIN_2_80MV
                                  : (g == 8) ? INA219_CONFIG_GAIN_8_320MV : INA219_CONFIG_GAIN_4_160MV;
        ina_update_config(0x1800, bits);
    }
    else if (strcmp(cmd, "avg") == 0)
    {
        char *a = strtok(NULL, " \t\r\n");
        int n = a ? atoi(a) : 1;
        uint16_t code = 0x3; // 12-bit, 1 sample
        for (int k = 1, c = 0x8; k <= 128; k *= 2, c++)
            if (k == n && n > 1)
                code = (uint16_t)c;
        ina_update_config(0x0078, (uint16_t)(code << 3));
    }
    else if (strcmp(cmd, "dyn") == 0)
    {
        char *a = strtok(NULL, " \t\r\n");
        if (a && strcmp(a, "on") == 0)
        {
            if (dynamic_load_enter())
            {
                g_app.ui_screen = UI_SCREEN_ACTION_DYNAMIC_LOAD;
                app_display_mark_dirty();
            }
        }
        else if (a && strcmp(a, "off") == 0)
        {
            dynamic_load_exit();
            g_app.ui_screen = UI_SCREEN_HOME;
            app_display_mark_dirty();
        }
        else if (a && strcmp(a, "up") == 0)
            dynamic_load_adjust(+1);
        else if (a && strcmp(a, "down") == 0)
            dynamic_load_adjust(-1);
    }
    else if (strcmp(cmd, "zero") == 0)
    {
        if (measurement_capture_zero())
            ESP_LOGI(TAG, "zero offset = %.2f mA", (double)measurement_ina_offset_mA());
    }
    else if (strcmp(cmd, "kcal") == 0)
    {
        char *a = strtok(NULL, " \t\r\n");
        if (!a)
            ESP_LOGW(TAG, "usage: kcal <idle mA read by the multimeter>");
        else
        {
            char *end = NULL;
            float mA = strtof(a, &end);
            if (end == a || *end != '\0')
                ESP_LOGW(TAG, "kcal: '%s' is not a number", a);
            else
                measurement_calibrate_k(mA);
        }
    }
    else if (strcmp(cmd, "raw") == 0)
    {
        char *a = strtok(NULL, " \t\r\n");
        char *b = strtok(NULL, " \t\r\n");
        raw_dump(a ? (uint32_t)strtoul(a, NULL, 10) : 0, b ? atoi(b) : 100);
    }
    else
    {
        ESP_LOGW(TAG, "unknown command '%s' (sweep | stop | scan <max_duty> <step>)", cmd);
    }
}

static void debug_console_task(void *arg)
{
    (void)arg;
    char line[64];
    size_t len = 0;
    for (;;)
    {
        uint8_t c;
        if (usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY) != 1)
            continue;
        if (c == '\n' || c == '\r')
        {
            if (len > 0)
            {
                line[len] = '\0';
                handle_line(line);
                len = 0;
            }
        }
        else if (len < sizeof(line) - 1)
        {
            line[len++] = (char)c;
        }
    }
}

void debug_console_start(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    if (usb_serial_jtag_driver_install(&cfg) != ESP_OK)
    {
        ESP_LOGW(TAG, "USB serial driver install failed, debug console disabled");
        return;
    }
    xTaskCreate(debug_console_task, "debug_console", 4096, NULL, 3, NULL);
    ESP_LOGI(TAG, "debug console ready: sweep | stop | scan <max_duty> <step>");
}
