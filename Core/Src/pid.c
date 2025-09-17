/**
 * @file pid.c
 * @brief Lightweight PID controller for Engineer Custom Controller
 * @author yammmyu
 * @date 09-2025
 *
 * @copyright Calibur Robotics (c) 2025
 */


#include "pid.h"

/**
 * @brief Initialize PID controller
 */
void pid_init(pid_t *pid,
              float Kp, float Ki, float Kd, float Kf,
              float i_max, float out_max,
              float dt, float dead_zone,
              float i_var_a, float i_var_b,
              float i_sep_threshold, pid_d_mode_t d_mode)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kf = Kf;
    pid->dt = dt;
    pid->dead_zone = dead_zone;
    pid->d_mode = d_mode;

    pid->i_out_max = i_max;
    pid->out_max = out_max;
    pid->i_var_a = i_var_a;
    pid->i_var_b = i_var_b;
    pid->i_sep_threshold = i_sep_threshold;

    pid->target = 0.0f;
    pid->now = 0.0f;
    pid->integral_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_now = 0.0f;
    pid->prev_target = 0.0f;
    pid->prev_out = 0.0f;
    pid->out = 0.0f;
}

void pid_set_target(pid_t *pid, float target) {
    pid->target = target;
}

void pid_set_now(pid_t *pid, float now) {
    pid->now = now;
}

void pid_reset_integral(pid_t *pid) {
    pid->integral_error = 0.0f;
}


/**
 * @brief Run one PID update Calculation
 * 
 * @return PID output
 */
void pid_tick(pid_t *pid)
{
    float P = 0.0f, I = 0.0f, D = 0.0f, F = 0.0f;
    
    float error = pid->target - pid->now;

    float abs_error = error > 0 ? error : -error;

    float speed_ratio;

    /* Dead zone */
    if (abs_error < pid->dead_zone)
    {
        error = 0.0f;
        abs_error = 0.0f;
    }

    /* Proportional */
    P = pid->Kp * error;

    // Integral with separation and variable speed
    if (pid->i_var_a == 0.0f && pid->i_var_b == 0.0f)
    {
        // Not Variable Speed Integral
        speed_ratio = 1.0f;
    }
    else
    {
        if (abs_error <= pid->i_var_a)
        {
            // Full gain zone
            speed_ratio = 1.0f;
        }
        else if (abs_error < pid->i_var_b)
        {
            // Fade-out zone (linear)
            speed_ratio = (pid->i_var_b - abs_error) / (pid->i_var_b - pid->i_var_a);
        }
        else
        {
            // No gain zone
            speed_ratio = 0.0f;
        }
    }

    // No Integral Separation or Within Separation Threshold
    if (pid->i_sep_threshold == 0.0f || (abs_error < pid->i_sep_threshold))
    {
        pid->integral_error += speed_ratio * error * pid->dt;

        /* Integral clamp */
        if (pid->i_out_max != 0.0f) {
            if (pid->integral_error > pid->i_out_max) pid->integral_error = pid->i_out_max;
            if (pid->integral_error < -pid->i_out_max) pid->integral_error = -pid->i_out_max;
        }

        I = pid->Ki * pid->integral_error;
    }

    /* Derivative */
    if (pid->d_mode == PID_D_FIRST_ENABLE) {

        // Derivative First
        D = pid->Kd * (pid->now - pid->prev_now) / pid->dt;
    } else {

        // No Derivative First
        D = pid->Kd * (error - pid->prev_error) / pid->dt;
    }

    /* Feedforward */
    float F = pid->Kf * (pid->target - pid->prev_target);

    /* Output sum */
    pid->out = P + I - D + F;

    /* Output clamp */
    if (pid->out_max != 0.0f) {
        if (pid->out > pid->out_max) pid->out = pid->out_max;
        if (pid->out < -pid->out_max) pid->out = -pid->out_max;
    }

    /* Save state */
    pid->prev_error = error;
    pid->prev_now = pid->now;
    pid->prev_target = pid->target;
    pid->prev_out = pid->out;
}

float pid_get_out(const pid_t *pid) {
    return pid->out;
}

float pid_get_integral(const pid_t *pid) {
    return pid->integral_error;
}
