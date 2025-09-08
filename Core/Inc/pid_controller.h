#ifndef PID_H
#define PID_H

typedef struct {
    float Kp, Ki, Kd;
    float integrator;
    float last_error;
    float last_measure;
    float out_min, out_max;
    float integ_min, integ_max;
    float I_deadband;
} pid_t;

float pid_update(pid_t *pid, float setpoint, float measurement, float dt);

#endif
