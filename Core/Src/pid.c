// PID.c
#include "PID.h"
#include "motor.h"
#include <math.h>  // NEW: For fmod in angle error calculation

typedef struct {
    float Kp, Ki, Kd;
    float integrator;
    float last_error;
    float last_measure;      // for derivative-on-measurement
    float out_min, out_max;  // absolute output limits
    float integ_min, integ_max; // integrator clamp
    float I_deadband;        // error magnitude above which integrate = 0 (0 to disable)
} pid_t;

/* Call every control cycle with dt in seconds */
float pid_update(pid_t *pid, float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    // Proportional
    float P = pid->Kp * error;

    // Integral (anti-windup with clamping and optional deadband)
    if ((pid->I_deadband > 0.0f) && (fabsf(error) > pid->I_deadband)) {
        // do not accumulate integral when error large
    } else {
        pid->integrator += pid->Ki * error * dt;
        // clamp integrator
        if (pid->integrator > pid->integ_max) pid->integrator = pid->integ_max;
        if (pid->integrator < pid->integ_min) pid->integrator = pid->integ_min;
    }
    float I = pid->integrator;

    // Derivative on measurement (reduces setpoint kick)
    float derivative = (measurement - pid->last_measure) / dt; // note sign
    float D = -pid->Kd * derivative;

    pid->last_error = error;
    pid->last_measure = measurement;

    float out = P + I + D;

    // output clamp
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    return out;
}
