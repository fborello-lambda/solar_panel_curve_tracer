#include "db.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static float s_x[DB_MAX_SAMPLES];
static float s_y[DB_MAX_SAMPLES];
static int s_head = -1;
static int s_count = 0;
static SemaphoreHandle_t s_mtx = NULL;

void db_init(void)
{
    if (s_mtx == NULL)
        s_mtx = xSemaphoreCreateMutex();
}

bool db_reset(void)
{
    if (s_mtx == NULL)
        db_init();

    if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(50)) != pdTRUE)
        return false;

    // Just reset head and count; data will be overwritten
    s_head = -1;
    s_count = 0;

    xSemaphoreGive(s_mtx);
    return true;
}

bool db_add(float x, float y)
{
    if (s_mtx == NULL)
        db_init();

    // Try to take mutex with short timeout to avoid blocking too long
    // The caller can retry if needed.
    if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(50)) != pdTRUE)
        return false;

    s_head = (s_head + 1) % DB_MAX_SAMPLES;
    s_x[s_head] = x;
    s_y[s_head] = y;
    if (s_count < DB_MAX_SAMPLES)
        s_count++;
    xSemaphoreGive(s_mtx);
    return true;
}

bool db_snapshot(float *x_out, float *y_out, size_t *count, size_t cap)
{
    if (s_mtx == NULL)
        db_init();
    if (!x_out || !y_out || cap == 0)
        return false;

    // Try to take mutex with short timeout to avoid blocking too long
    // The caller can retry if needed.
    if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(50)) != pdTRUE)
        return false;
    *count = s_count;
    if (*count > cap)
        *count = cap;
    if (*count > 0)
    {
        int oldest = (s_head - *count + 1 + DB_MAX_SAMPLES) % DB_MAX_SAMPLES;
        for (int i = 0; i < *count; ++i)
        {
            int idx = (oldest + i) % DB_MAX_SAMPLES;
            x_out[i] = s_x[idx];
            y_out[i] = s_y[idx];
        }
    }
    xSemaphoreGive(s_mtx);
    return true;
}
