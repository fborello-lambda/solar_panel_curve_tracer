#pragma once

#include <stdint.h>

void ui_init_state(void);
void ui_on_rotate(int dir);
void ui_on_button(void);
void ui_render_display_frame(uint8_t *fb);
