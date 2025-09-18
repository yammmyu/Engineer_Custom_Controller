// pid_params.c
#include "pid_params.h"

/*
 * Defaults — adjust for your machine.
 * These variables are intentionally non-const so you can edit them in the debugger.
 */

#include "pid_params.h"

/*
 * Defaults — adjust for your machine.
 * These variables are intentionally non-const so you can edit them in the debugger.
 */

motor_pid_cfg_t motor_pid_cfg[MOTOR_COUNT] = {
    // motor0
    {
        .angle  = { 20.0f, 0.0f, 0.5f, 0.0f, 1000, 2000, 0.001f, 0.0f, 0,0,0,0 },
        .torque = { 0.8f,  0.01f, 0.0f, 0.0f,  300, 1000, 0.001f, 0.0f, 0,0,0,0 }
    },

    // motor1
    {
        .angle  = { 20.0f, 0.0f, 0.5f, 0.0f, 1000, 2000, 0.001f, 0.0f, 0,0,0,0 },
        .torque = { 0.8f,  0.01f, 0.0f, 0.0f,  300, 1000, 0.001f, 0.0f, 0,0,0,0 }
    },

    // motor2
    {
        .angle  = { 20.0f, 0.0f, 0.5f, 0.0f, 1000, 2000, 0.001f, 0.0f, 0,0,0,0 },
        .torque = { 0.8f,  0.01f, 0.0f, 0.0f,  300, 1000, 0.001f, 0.0f, 0,0,0,0 }
    },

    // motor3
    {
        .angle  = { 6.0f,  0.0f, 0.2f, 0.0f, 1000, 2000, 0.001f, 0.0f, 0,0,0,0 },
        .torque = { 0.3f,  0.01f, 0.0f, 0.0f,  300, 2000, 0.001f, 0.0f, 0,0,0,0 }
    },

    // motor4
    {
        .angle  = { 6.0f,  0.0f, 0.2f, 0.0f, 1000, 2000, 0.001f, 0.0f, 0,0,0,0 },
        .torque = { 0.3f,  0.01f, 0.0f, 0.0f,  300, 2000, 0.001f, 0.0f, 0,0,0,0 }
    },

    // motor5
    {
        .angle  = { 6.0f,  0.0f, 0.2f, 0.0f, 1000, 2000, 0.001f, 0.0f, 0,0,0,0 },
        .torque = { 0.3f,  0.01f, 0.0f, 0.0f,  300, 2000, 0.001f, 0.0f, 0,0,0,0 }
    }
};
