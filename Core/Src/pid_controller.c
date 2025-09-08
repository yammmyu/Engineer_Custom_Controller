#include "pid_controller.h"
#include <math.h>

volatile float error = 0.0;

void pid_init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float out_min, float out_max,
              float integ_min, float integ_max,
              float I_deadband)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integ_min = integ_min;
    pid->integ_max = integ_max;

    pid->I_deadband = I_deadband;
}

float clampf(float value, float min_val, float max_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

float pid_update(PID_t *pid, float setpoint, float measurement, float dt)
{
	if (fabs(setpoint - measurement) > 180.0) {
		if (setpoint > measurement) {
			error = (setpoint - measurement) - 360;
		}else{
			error = (setpoint - measurement) + 360;
		}
	}else{
		error = setpoint - measurement;
	}

    // Proportional
    float P_out = pid->Kp * error;

    // Integral with deadband
    if (fabsf(error) < pid->I_deadband) {
        pid->integrator += pid->Ki * error * dt;
        pid->integrator = clampf(pid->integrator, pid->integ_min, pid->integ_max);
    }

    // Derivative on measurement
    float derivative = (measurement - pid->prev_measurement) / dt;
    float D_out = -pid->Kd * derivative;

    float output = P_out + pid->integrator + D_out;

    // Clamp total output
    output = clampf(output, pid->out_min, pid->out_max);

    // Save for next iteration
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return output;
}

int16_t voltage_to_can(float demand)
{
    // demand expected in [-1, 1], scale to [-25000, 25000]
    if (demand > 1.0f) demand = 1.0f;
    if (demand < -1.0f) demand = -1.0f;

    return (int16_t)(demand * 25000.0f);
}
