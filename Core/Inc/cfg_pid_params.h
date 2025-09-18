// pid_params.h
#ifndef PID_PARAMS_H
#define PID_PARAMS_H

#include <alg_pid.h>   // your existing pid_t and pid API
#include <stdint.h>

#define MOTOR_COUNT 6

/**
 * @brief Blueprint for PID configuration parameters
 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float i_out_max;
    float out_max;
    float dt;
    float dead_zone;
    float i_var_a;
    float i_var_b;
    float i_sep_threshold;
    int   d_first;         // 0 = derivative on error, 1 = derivative on measurement
} pid_cfg_t;

/**
 * @brief Holds both angle-loop and torque-loop PID configs for one motor
 */
typedef struct {
    pid_cfg_t angle;   ///< Angle PID parameters
    pid_cfg_t torque;  ///< Torque PID parameters
} motor_pid_cfg_t;

/* ---------------- Extern Declarations ---------------- */

// Global array defined in pid_params.c
extern motor_pid_cfg_t motor_pid_cfg[MOTOR_COUNT];

/* Outer loop (angle -> torque) */
extern pid_cfg_t pid_angle_cfg[MOTOR_COUNT];

/* Inner loop (torque -> output) */
extern pid_cfg_t pid_torque_cfg[MOTOR_COUNT];

#endif // PID_PARAMS_H
