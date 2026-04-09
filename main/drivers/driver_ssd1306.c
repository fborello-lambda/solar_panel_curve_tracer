#include "driver_ssd1306.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 3
#endif

#include <ctype.h>
#include <string.h>

#define SSD1306_CTRL_CMD 0x00
#define SSD1306_CTRL_DATA 0x40
#define SSD1306_TIMEOUT_MS 1000

static esp_err_t ssd1306_write_cmd(const ssd1306_t *display, uint8_t cmd)
{
    uint8_t payload[2] = {SSD1306_CTRL_CMD, cmd};
    return i2c_master_transmit(display->dev, payload, sizeof(payload), SSD1306_TIMEOUT_MS);
}

static esp_err_t ssd1306_write_data(const ssd1306_t *display, const uint8_t *data, size_t len)
{
    if (!data || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t chunk[17];
    chunk[0] = SSD1306_CTRL_DATA;

    size_t offset = 0;
    while (offset < len)
    {
        size_t n = len - offset;
        if (n > 16)
        {
            n = 16;
        }

        memcpy(&chunk[1], data + offset, n);
        esp_err_t ret = i2c_master_transmit(display->dev, chunk, n + 1, SSD1306_TIMEOUT_MS);
        if (ret != ESP_OK)
        {
            return ret;
        }
        offset += n;
    }

    return ESP_OK;
}

esp_err_t ssd1306_init_on_bus(ssd1306_t *display, i2c_master_bus_handle_t bus, uint8_t i2c_addr, uint32_t scl_hz)
{
    if (!display || !bus)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(display, 0, sizeof(*display));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = scl_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &display->dev);
    if (ret != ESP_OK)
    {
        return ret;
    }

    display->width = SSD1306_WIDTH;
    display->height = SSD1306_HEIGHT;
    display->column_offset = 0;

    const uint8_t init_cmds[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x00,
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0x7F,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0x2E,
        0xAF,
    };

    for (size_t i = 0; i < sizeof(init_cmds); i++)
    {
        ret = ssd1306_write_cmd(display, init_cmds[i]);
        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    return ssd1306_clear(display);
}

esp_err_t ssd1306_set_rotation(const ssd1306_t *display, bool rotate_180)
{
    if (!display || !display->dev)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ssd1306_write_cmd(display, rotate_180 ? 0xA0 : 0xA1);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ssd1306_write_cmd(display, rotate_180 ? 0xC0 : 0xC8);
}

void ssd1306_set_column_offset(ssd1306_t *display, uint8_t column_offset)
{
    if (!display)
    {
        return;
    }
    display->column_offset = column_offset;
}

esp_err_t ssd1306_clear(const ssd1306_t *display)
{
    uint8_t blank[SSD1306_FB_SIZE] = {0};
    return ssd1306_flush(display, blank, sizeof(blank));
}

esp_err_t ssd1306_flush(const ssd1306_t *display, const uint8_t *fb, size_t len)
{
    if (!display || !display->dev || !fb || len != SSD1306_FB_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t col_start = display->column_offset;
    uint8_t col_end = (uint8_t)(display->column_offset + SSD1306_WIDTH - 1);

    esp_err_t ret = ssd1306_write_cmd(display, 0x21);
    if (ret != ESP_OK)
        return ret;
    ret = ssd1306_write_cmd(display, col_start);
    if (ret != ESP_OK)
        return ret;
    ret = ssd1306_write_cmd(display, col_end);
    if (ret != ESP_OK)
        return ret;

    ret = ssd1306_write_cmd(display, 0x22);
    if (ret != ESP_OK)
        return ret;
    ret = ssd1306_write_cmd(display, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = ssd1306_write_cmd(display, 0x07);
    if (ret != ESP_OK)
        return ret;

    return ssd1306_write_data(display, fb, len);
}

void ssd1306_fb_clear(uint8_t *fb, bool on)
{
    memset(fb, on ? 0xFF : 0x00, SSD1306_FB_SIZE);
}

void ssd1306_fb_set_pixel(uint8_t *fb, int x, int y, bool on)
{
    if (!fb || x < 0 || y < 0 || x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        return;
    }

    const int page = y / 8;
    const int index = page * SSD1306_WIDTH + x;
    const uint8_t mask = (uint8_t)(1U << (y % 8));

    if (on)
    {
        fb[index] |= mask;
    }
    else
    {
        fb[index] &= (uint8_t)~mask;
    }
}

void ssd1306_fb_draw_rect(uint8_t *fb, int x, int y, int w, int h, bool fill, bool on)
{
    if (!fb || w <= 0 || h <= 0)
    {
        return;
    }

    for (int yy = y; yy < y + h; yy++)
    {
        for (int xx = x; xx < x + w; xx++)
        {
            bool edge = (xx == x) || (xx == x + w - 1) || (yy == y) || (yy == y + h - 1);
            if (fill || edge)
            {
                ssd1306_fb_set_pixel(fb, xx, yy, on);
            }
        }
    }
}

static bool glyph_for_char(char c, uint8_t out[5])
{
    const char uc = (char)toupper((unsigned char)c);
    switch (uc)
    {
    case 'A':
        memcpy(out, (uint8_t[]){0x7E, 0x11, 0x11, 0x11, 0x7E}, 5);
        return true;
    case 'B':
        memcpy(out, (uint8_t[]){0x7F, 0x49, 0x49, 0x49, 0x36}, 5);
        return true;
    case 'C':
        memcpy(out, (uint8_t[]){0x3E, 0x41, 0x41, 0x41, 0x22}, 5);
        return true;
    case 'D':
        memcpy(out, (uint8_t[]){0x7F, 0x41, 0x41, 0x22, 0x1C}, 5);
        return true;
    case 'E':
        memcpy(out, (uint8_t[]){0x7F, 0x49, 0x49, 0x49, 0x41}, 5);
        return true;
    case 'F':
        memcpy(out, (uint8_t[]){0x7F, 0x09, 0x09, 0x09, 0x01}, 5);
        return true;
    case 'G':
        memcpy(out, (uint8_t[]){0x3E, 0x41, 0x49, 0x49, 0x3A}, 5);
        return true;
    case 'H':
        memcpy(out, (uint8_t[]){0x7F, 0x08, 0x08, 0x08, 0x7F}, 5);
        return true;
    case 'I':
        memcpy(out, (uint8_t[]){0x41, 0x41, 0x7F, 0x41, 0x41}, 5);
        return true;
    case 'J':
        memcpy(out, (uint8_t[]){0x20, 0x40, 0x41, 0x3F, 0x01}, 5);
        return true;
    case 'K':
        memcpy(out, (uint8_t[]){0x7F, 0x08, 0x14, 0x22, 0x41}, 5);
        return true;
    case 'L':
        memcpy(out, (uint8_t[]){0x7F, 0x40, 0x40, 0x40, 0x40}, 5);
        return true;
    case 'M':
        memcpy(out, (uint8_t[]){0x7F, 0x02, 0x0C, 0x02, 0x7F}, 5);
        return true;
    case 'N':
        memcpy(out, (uint8_t[]){0x7F, 0x04, 0x08, 0x10, 0x7F}, 5);
        return true;
    case 'O':
        memcpy(out, (uint8_t[]){0x3E, 0x41, 0x41, 0x41, 0x3E}, 5);
        return true;
    case 'P':
        memcpy(out, (uint8_t[]){0x7F, 0x09, 0x09, 0x09, 0x06}, 5);
        return true;
    case 'Q':
        memcpy(out, (uint8_t[]){0x3E, 0x41, 0x51, 0x21, 0x5E}, 5);
        return true;
    case 'R':
        memcpy(out, (uint8_t[]){0x7F, 0x09, 0x19, 0x29, 0x46}, 5);
        return true;
    case 'S':
        memcpy(out, (uint8_t[]){0x46, 0x49, 0x49, 0x49, 0x31}, 5);
        return true;
    case 'T':
        memcpy(out, (uint8_t[]){0x01, 0x01, 0x7F, 0x01, 0x01}, 5);
        return true;
    case 'U':
        memcpy(out, (uint8_t[]){0x3F, 0x40, 0x40, 0x40, 0x3F}, 5);
        return true;
    case 'V':
        memcpy(out, (uint8_t[]){0x1F, 0x20, 0x40, 0x20, 0x1F}, 5);
        return true;
    case 'W':
        memcpy(out, (uint8_t[]){0x7F, 0x20, 0x18, 0x20, 0x7F}, 5);
        return true;
    case 'X':
        memcpy(out, (uint8_t[]){0x63, 0x14, 0x08, 0x14, 0x63}, 5);
        return true;
    case 'Y':
        memcpy(out, (uint8_t[]){0x03, 0x04, 0x78, 0x04, 0x03}, 5);
        return true;
    case 'Z':
        memcpy(out, (uint8_t[]){0x61, 0x51, 0x49, 0x45, 0x43}, 5);
        return true;
    case '0':
        memcpy(out, (uint8_t[]){0x3E, 0x45, 0x49, 0x51, 0x3E}, 5);
        return true;
    case '1':
        memcpy(out, (uint8_t[]){0x00, 0x21, 0x7F, 0x01, 0x00}, 5);
        return true;
    case '2':
        memcpy(out, (uint8_t[]){0x23, 0x45, 0x49, 0x51, 0x21}, 5);
        return true;
    case '3':
        memcpy(out, (uint8_t[]){0x42, 0x41, 0x51, 0x69, 0x46}, 5);
        return true;
    case '4':
        memcpy(out, (uint8_t[]){0x18, 0x28, 0x48, 0x7F, 0x08}, 5);
        return true;
    case '5':
        memcpy(out, (uint8_t[]){0x72, 0x51, 0x51, 0x51, 0x4E}, 5);
        return true;
    case '6':
        memcpy(out, (uint8_t[]){0x1E, 0x29, 0x49, 0x49, 0x06}, 5);
        return true;
    case '7':
        memcpy(out, (uint8_t[]){0x40, 0x47, 0x48, 0x50, 0x60}, 5);
        return true;
    case '8':
        memcpy(out, (uint8_t[]){0x36, 0x49, 0x49, 0x49, 0x36}, 5);
        return true;
    case '9':
        memcpy(out, (uint8_t[]){0x30, 0x49, 0x49, 0x4A, 0x3C}, 5);
        return true;
    case '_':
        memcpy(out, (uint8_t[]){0x80, 0x80, 0x80, 0x80, 0x80}, 5);
        return true;
    case '-':
        memcpy(out, (uint8_t[]){0x08, 0x08, 0x08, 0x08, 0x08}, 5);
        return true;
    case '.':
        memcpy(out, (uint8_t[]){0x00, 0x60, 0x60, 0x00, 0x00}, 5);
        return true;
    case ':':
        memcpy(out, (uint8_t[]){0x00, 0x36, 0x36, 0x00, 0x00}, 5);
        return true;
    case '/':
        memcpy(out, (uint8_t[]){0x03, 0x0C, 0x10, 0x60, 0x80}, 5);
        return true;
    case ' ':
        memcpy(out, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00}, 5);
        return true;
    default:
        memcpy(out, (uint8_t[]){0x7F, 0x41, 0x41, 0x41, 0x7F}, 5);
        return false;
    }
}

static void ssd1306_fb_draw_char(uint8_t *fb, int x, int y, char c)
{
    uint8_t glyph[5];
    glyph_for_char(c, glyph);

    for (int col = 0; col < 5; col++)
    {
        const uint8_t bits = glyph[col];
        for (int row = 0; row < 8; row++)
        {
            const bool on = ((bits >> row) & 0x01U) != 0;
            if (on)
            {
                ssd1306_fb_set_pixel(fb, x + col, y + row, true);
            }
        }
    }
}

void ssd1306_fb_draw_text(uint8_t *fb, int x, int y, const char *txt)
{
    if (!fb || !txt)
    {
        return;
    }

    int x_cur = x;
    int y_cur = y;

    for (size_t i = 0; txt[i] != '\0'; i++)
    {
        if (txt[i] == '\n')
        {
            y_cur += 10;
            x_cur = x;
            continue;
        }

        ssd1306_fb_draw_char(fb, x_cur, y_cur, txt[i]);
        x_cur += 6;

        if (x_cur > SSD1306_WIDTH - 6)
        {
            y_cur += 10;
            x_cur = x;
        }

        if (y_cur > SSD1306_HEIGHT - 8)
        {
            break;
        }
    }
}
