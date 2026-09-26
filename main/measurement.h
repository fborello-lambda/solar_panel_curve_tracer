#pragma once

#include <stdbool.h>

typedef enum
{
    CURVE_PRODUCER_REAL = 0,
    CURVE_PRODUCER_DUMMY,
} curve_producer_mode_t;

bool measurement_is_running(void);
bool measurement_request(bool start);
bool measurement_init_load_control_hw(void);

void measurement_set_producer_mode(curve_producer_mode_t mode);
curve_producer_mode_t measurement_get_producer_mode(void);
const char *measurement_get_producer_mode_label(void);
