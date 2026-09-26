#pragma once

#include <stdbool.h>

// Manual, encoder-driven electronic load. Independent of the auto-ranged
// sweep in measurement.c: the operator dials the duty directly and watches
// live current/power, used for bench experiments rather than curve tracing.

void dynamic_load_adjust(int dir);
void dynamic_load_update_measured(void);
// Returns true if dynamic load mode was actually entered (refused e.g. while
// a measurement is running).
bool dynamic_load_enter(void);
void dynamic_load_exit(void);
