#pragma once

#include <stdbool.h>

typedef enum
{
    CURVE_PRODUCER_REAL = 0,
    CURVE_PRODUCER_DUMMY,
} curve_producer_mode_t;

bool measurement_is_running(void);
bool measurement_request(bool start);
bool measurement_init_load_control_hw(bool strict_mode);

void measurement_set_producer_mode(curve_producer_mode_t mode);
curve_producer_mode_t measurement_get_producer_mode(void);
const char *measurement_get_producer_mode_label(void);

void dynamic_load_adjust(int dir);
void dynamic_load_update_measured(void);
void dynamic_load_enter(void);
void dynamic_load_exit(void);
