#include "stm32f4xx_hal.h"
#include "timebase.h"

static uint32_t last_tick = 0;

float get_dt(void)
{
	uint32_t now = HAL_GetTick();
	float dt = (now - last_tick) / 1000.0f;
	last_tick = now;
	return dt;
}
