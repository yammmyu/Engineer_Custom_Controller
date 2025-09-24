#include "cfg_pid_params.h"
#include "dvc_motor_config.h"
#include "stm32f4xx_hal.h"


/*
 * Defaults — adjust for your machine.
 * These variables are intentionally non-const so you can edit them in the debugger.
 */

//        pid_init(&motors[i].PID_Angle,
//                 motor_pid_cfg[i].angle.Kp,
//                 motor_pid_cfg[i].angle.Ki,
//                 motor_pid_cfg[i].angle.Kf,
//				 motor_pid_cfg[i].angle.Kd,
//                 motor_pid_cfg[i].angle.i_out_max,
//                 motor_pid_cfg[i].angle.out_max,
//                 motor_pid_cfg[i].angle.dt,
//                 motor_pid_cfg[i].angle.dead_zone,
//                 motor_pid_cfg[i].angle.i_var_a,
//                 motor_pid_cfg[i].angle.i_var_b,
//                 motor_pid_cfg[i].angle.i_sep_threshold,
//                 motor_pid_cfg[i].angle.d_first);


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
        .angle  = { 20000.0f, 1.0f, 0.0f, 0.0f, 0, 25000, 0.0f, 0.0f, 0,0,0,0 }, //Currently only using single loop
        .torque = { 1.0f, 0.0f, 0.0f, 0.0f, 100, 25000, 0.0f, 20.0f, 0,0,0,0 }
//        .angle  = { 10000.0f, 3000.0f, 0.0f, 0.0f, 1, 25000, 0.0f, 0.0f, 0,0,0,0 }, //Currently only using single loop
//        .torque = { 1.0f, 0.0f, 0.0f, 0.0f, 100, 25000, 0.0f, 20.0f, 0,0,0,0 }
    },

    // motor4
    {
        .angle  = { 30000.0f, 550.0f, 0.0f, 0.0f, 1000, 25000, 0.0f, 0.0f, 0,0,0,0 }, //Currently only using single loop
        .torque = { 1.0f, 0.0f, 0.0f, 0.0f, 100, 25000, 0.0f, 20.0f, 0,0,0,0 }
    },

    // motor5
    {
        .angle  = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 },
        .torque = { 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0,0,0,0 }
    }
};
