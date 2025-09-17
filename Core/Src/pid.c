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

    pid->i_max = i_max;
    pid->out_max = out_max;
    pid->i_var_a = i_var_a;
    pid->i_var_b = i_var_b;
    pid->i_sep_threshold = i_sep_threshold;

    pid->target = 0.0f;
    pid->now = 0.0f;
    pid->integral = 0.0f;
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
    pid->integral = 0.0f;
}

/**
 * @brief Run one PID update step
 */
void pid_tick(pid_t *pid)
{
    float error = pid->target - pid->now;

    /* Dead zone */
    if (pid->dead_zone > 0.0f && (error > -pid->dead_zone && error < pid->dead_zone)) {
        pid->out = 0.0f;
        return;
    }

    /* Proportional */
    float P = pid->Kp * error;

    /* Integral with separation and variable speed */
    if (pid->i_sep_threshold == 0.0f || (error > -pid->i_sep_threshold && error < pid->i_sep_threshold)) {
        pid->integral += pid->Ki * error * pid->dt;

        /* Variable speed integral scaling */
        if (pid->i_var_a > 0 && pid->i_var_b > pid->i_var_a) {
            if (error > pid->i_var_a && error < pid->i_var_b) {
                float scale = (pid->i_var_b - error) / (pid->i_var_b - pid->i_var_a);
                pid->integral *= scale;
            } else if (error >= pid->i_var_b) {
                pid->integral = 0.0f;
            }
        }

        /* Integral clamp */
        if (pid->i_max > 0.0f) {
            if (pid->integral > pid->i_max) pid->integral = pid->i_max;
            if (pid->integral < -pid->i_max) pid->integral = -pid->i_max;
        }
    }

    /* Derivative */
    float derivative;
    if (pid->d_mode == PID_D_FIRST_ENABLE) {
        derivative = (pid->now - pid->prev_now) / pid->dt;
    } else {
        derivative = (error - pid->prev_error) / pid->dt;
    }
    float D = pid->Kd * derivative;

    /* Feedforward */
    float F = pid->Kf * pid->target;

    /* Output sum */
    pid->out = P + pid->integral - D + F;

    /* Output clamp */
    if (pid->out_max > 0.0f) {
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
    return pid->integral;
}
