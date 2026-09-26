#include "quadrature.h"

/*
 * Classic Gray-code valid-transition table, indexed by
 * (previous_state << 2) | current_state, where each state is (clk << 1 | dt)
 * in {0,1,2,3}. A legal single-step quadrature transition (00->01->11->10->00
 * for one direction, the reverse for the other) yields +1/-1; anything else
 * (a repeated state, a two-bit jump, or bounce) yields 0 and is ignored.
 */
static const int8_t QDEC_TABLE[16] = {
    /* prev=00 */ 0, -1, 1, 0,
    /* prev=01 */ 1, 0, 0, -1,
    /* prev=10 */ -1, 0, 0, 1,
    /* prev=11 */ 0, 1, -1, 0,
};

void qdec_init(qdec_t *q, int clk_level, int dt_level)
{
    if (!q)
    {
        return;
    }
    q->state = (uint8_t)(((clk_level ? 1 : 0) << 1) | (dt_level ? 1 : 0));
    q->accum = 0;
}

int qdec_update(qdec_t *q, int clk_level, int dt_level, uint32_t counts_per_step)
{
    if (!q)
    {
        return 0;
    }

    uint8_t curr = (uint8_t)(((clk_level ? 1 : 0) << 1) | (dt_level ? 1 : 0));
    uint8_t idx = (uint8_t)((q->state << 2) | curr);
    int8_t delta = QDEC_TABLE[idx & 0x0F];
    q->state = curr;

    if (delta == 0)
    {
        return 0;
    }

    /* A direction reversal resets the accumulator so flipping direction
     * responds immediately instead of fighting the previous partial step. */
    if ((delta > 0 && q->accum < 0) || (delta < 0 && q->accum > 0))
    {
        q->accum = 0;
    }
    q->accum += delta;

    if (counts_per_step == 0)
    {
        counts_per_step = 4;
    }

    if (q->accum >= (int32_t)counts_per_step)
    {
        q->accum = 0;
        return 1;
    }
    if (q->accum <= -(int32_t)counts_per_step)
    {
        q->accum = 0;
        return -1;
    }
    return 0;
}
