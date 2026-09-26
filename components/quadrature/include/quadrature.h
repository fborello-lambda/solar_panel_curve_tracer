#ifndef QUADRATURE_H
#define QUADRATURE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Pure quadrature (Gray-code) decoder for a two-channel rotary encoder
 * (CLK/DT, or A/B). No hardware or FreeRTOS dependencies, so this builds and
 * is unit-tested on the ESP-IDF `linux` target (see test/host).
 *
 * Usage: call qdec_init() once with the encoder's resting levels, then call
 * qdec_update() on every edge of either channel (from an ISR or a polling
 * loop) with the freshly read levels of both channels. Invalid transitions
 * (contact bounce, a level that doesn't follow a valid Gray-code step from
 * the last one) are silently absorbed and contribute nothing.
 */
typedef struct
{
    uint8_t state; /* last (clk<<1 | dt) reading, 0..3 */
    int32_t accum; /* accumulated valid quarter-step transitions */
} qdec_t;

void qdec_init(qdec_t *q, int clk_level, int dt_level);

/*
 * Feed one new (clk, dt) level pair. counts_per_step is how many valid
 * quarter-step transitions accumulate into one emitted step (a full
 * quadrature cycle is 4 transitions; the driver's default is
 * counts_per_step=4, i.e. one emitted step per full cycle).
 *
 * Returns +1 for a completed clockwise step, -1 for counter-clockwise, or 0
 * if no step is ready yet (including when the transition was invalid/bounce
 * and was ignored).
 */
int qdec_update(qdec_t *q, int clk_level, int dt_level, uint32_t counts_per_step);

#ifdef __cplusplus
}
#endif

#endif
