/**
 * @file pid.h
 * @brief Lightweight PID controller for Engineer Custom Controller
 * @author yammmyu
 * @date 09-2025
 *
 * @copyright Caliber Robotics (c) 2025
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Derivative computation method
 */
typedef enum {
    PID_D_FIRST_DISABLE = 0, /**< Derivative on error */
    PID_D_FIRST_ENABLE       /**< Derivative on measurement (derivative first) */
} pid_d_mode_t;

/**
 * @brief PID controller structure
 *
 * All members are public for simplicity, but should normally be
 * accessed via the provided functions.
 */
typedef struct {
    /* Configurable constants */
    float Kp;	//P
    float Ki;	//I
    float Kd;	//D
    float Kf;	//Feed forward Gain
    float dt;	//Sample Time - Interruption
    float dead_zone; //Error Deadzone - disables PID if within
    pid_d_mode_t d_mode;

    float i_max;               /**< Integral limit (0 = disabled) */
    float out_max;             /**< Output limit (0 = disabled) */
    float i_var_a;             /**< Variable speed integral A threshold */
    float i_var_b;             /**< Variable speed integral B threshold */
    float i_sep_threshold;     /**< Integral separation threshold (0 = disabled) */

    /* State variables */
    float target;
    float now;
    float integral;
    float prev_error;
    float prev_now;
    float prev_target;
    float prev_out;

    /* Output */
    float out;
} pid_t;

/* Initialization */
void pid_init(pid_t *pid,
              float Kp, float Ki, float Kd, float Kf,
              float i_max, float out_max,
              float dt, float dead_zone,
              float i_var_a, float i_var_b,
              float i_sep_threshold, pid_d_mode_t d_mode);

/* Operation */
void pid_set_target(pid_t *pid, float target);
void pid_set_now(pid_t *pid, float now);
void pid_reset_integral(pid_t *pid);
void pid_tick(pid_t *pid);     /**< Run one update (call from ISR or loop) */

/* Accessors */
float pid_get_out(const pid_t *pid);
float pid_get_integral(const pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
