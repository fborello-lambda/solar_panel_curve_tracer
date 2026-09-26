#include "debug_console.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <driver/usb_serial_jtag.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "measurement.h"

static const char *TAG = "DEBUG";

// Line commands over the USB serial port, for bench debugging:
//   sweep            start a REAL sweep (same as START TRACE)
//   stop             stop a running sweep
//   scan <max> <st>  raw fixed-step load scan, one CSV line per step
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
