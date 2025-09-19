// dvc_timebase.c
#include "stm32f4xx_hal.h"
#include "dvc_timebase.h"

#define MS_TO_S 0.001f
static uint32_t last_tick = 0;

void init_timebase(void) {
    last_tick = HAL_GetTick(); // start from current time
}

float get_dt(void) {
    uint32_t now = HAL_GetTick();
    float dt = (now - last_tick) * MS_TO_S; // ms → seconds
    last_tick = now;
    if (dt <= 0.0f) dt = 1e-6f; // safety clamp
    return dt;
}
