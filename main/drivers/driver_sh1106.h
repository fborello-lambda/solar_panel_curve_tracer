#ifndef DRIVER_SH1106_H
#define DRIVER_SH1106_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <esp_err.h>
#include <driver/i2c_master.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SH1106_DEFAULT_I2C_ADDR 0x3C
#define SH1106_ALT_I2C_ADDR 0x3D
#define SH1106_WIDTH 128
#define SH1106_HEIGHT 64
#define SH1106_FB_SIZE (SH1106_WIDTH * SH1106_HEIGHT / 8)

typedef struct
{
    i2c_master_dev_handle_t dev;
    uint8_t width;
    uint8_t height;
    uint8_t column_offset;
} sh1106_t;

esp_err_t sh1106_init_on_bus(sh1106_t *display, i2c_master_bus_handle_t bus, uint8_t i2c_addr, uint32_t scl_hz);
esp_err_t sh1106_set_rotation(const sh1106_t *display, bool rotate_180);
void sh1106_set_column_offset(sh1106_t *display, uint8_t column_offset);
esp_err_t sh1106_clear(const sh1106_t *display);
esp_err_t sh1106_flush(const sh1106_t *display, const uint8_t *fb, size_t len);

void sh1106_fb_clear(uint8_t *fb, bool on);
void sh1106_fb_set_pixel(uint8_t *fb, int x, int y, bool on);
void sh1106_fb_draw_rect(uint8_t *fb, int x, int y, int w, int h, bool fill, bool on);
void sh1106_fb_draw_text(uint8_t *fb, int x, int y, const char *txt);

#ifdef __cplusplus
}
#endif

#endif
