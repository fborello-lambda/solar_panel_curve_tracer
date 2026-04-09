#ifndef DRIVER_SSD1306_H
#define DRIVER_SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <esp_err.h>
#include <driver/i2c_master.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SSD1306_DEFAULT_I2C_ADDR 0x3C
#define SSD1306_ALT_I2C_ADDR 0x3D
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_FB_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

typedef struct
{
    i2c_master_dev_handle_t dev;
    uint8_t width;
    uint8_t height;
    uint8_t column_offset;
} ssd1306_t;

esp_err_t ssd1306_init_on_bus(ssd1306_t *display, i2c_master_bus_handle_t bus, uint8_t i2c_addr, uint32_t scl_hz);
esp_err_t ssd1306_set_rotation(const ssd1306_t *display, bool rotate_180);
void ssd1306_set_column_offset(ssd1306_t *display, uint8_t column_offset);
esp_err_t ssd1306_clear(const ssd1306_t *display);
esp_err_t ssd1306_flush(const ssd1306_t *display, const uint8_t *fb, size_t len);

void ssd1306_fb_clear(uint8_t *fb, bool on);
void ssd1306_fb_set_pixel(uint8_t *fb, int x, int y, bool on);
void ssd1306_fb_draw_rect(uint8_t *fb, int x, int y, int w, int h, bool fill, bool on);
void ssd1306_fb_draw_text(uint8_t *fb, int x, int y, const char *txt);

#ifdef __cplusplus
}
#endif

#endif
