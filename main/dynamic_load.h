#pragma once

#include <stdbool.h>

// Manual, encoder-driven electronic load. Independent of the auto-ranged
// sweep in measurement.c: the operator dials the duty directly and watches
// live current/power, used for bench experiments rather than curve tracing.

// dynamic_range_state values. A range request is only accepted while the
// screen is idle; display_task first renders "RANGING" (REQUESTED ->
// RUNNING), then runs the probe on its next pass.
#define DYNAMIC_RANGE_IDLE 0
#define DYNAMIC_RANGE_REQUESTED 1
#define DYNAMIC_RANGE_RUNNING 2

// Moves the knob position one step (dir = +1 / -1) out of DYNAMIC_LOAD_POSITIONS.
void dynamic_load_adjust(int dir);
// Asks for a range probe: the load ramps up until the source collapses (or
// the duty / power cap), and that duty becomes the top knob position.
void dynamic_load_request_range(void);
void dynamic_load_update_measured(void);
// Returns true if dynamic load mode was actually entered (refused e.g. while
// a measurement is running).
bool dynamic_load_enter(void);
void dynamic_load_exit(void);
