#include "cfg_pid_params.h"
#include "dvc_motor_config.h"
#include "stm32f4xx_hal.h"


/*
 * Defaults — adjust for your machine.
 * These variables are intentionally non-const so you can edit them in the debugger.
 */

motor_pid_cfg_t motor_pid_cfg[MOTOR_COUNT] = {
    // motor0
    {
        .angle  = { 1.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 },
        .torque = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 }
    },

    // motor1
    {
        .angle  = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 },
        .torque = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 }
    },

    // motor2
    {
        .angle  = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 },
        .torque = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 }
    },

    // motor3
    {
        .angle  = { 2000.0f, 0.0f, 0.0f, 0.0f, 0, 25000, 0.0f, 0.0f, 0,0,0,0 },
        .torque = { 1.0f, 0.0f, 0.0f, 0.0f, 0, 25000, 0.0f, 0.0f, 0,0,0,0 }
    },

    // motor4
    {
        .angle  = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 },
        .torque = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 }
    },

    // motor5
    {
        .angle  = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 },
        .torque = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 }
    }
};
