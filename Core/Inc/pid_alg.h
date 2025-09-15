#ifndef PID_ALG_H
#define PID_ALG_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integrator;
    float prev_error;
    float prev_measurement;

    float out_min;
    float out_max;

    float integ_min;
    float integ_max;

    float I_deadband;
} PID_t;

/**
 * @brief Initialize a PID controller with given parameters.
 */
void pid_init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float out_min, float out_max,
              float integ_min, float integ_max,
              float I_deadband);

/**
 * @brief Update a PID controller.
 * @param pid Pointer to PID struct
 * @param setpoint Desired value
 * @param measurement Actual measured value
 * @param dt Time step (s)
 * @return Control signal
 */
float pid_update(PID_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief Clamp helper
 */
float clampf(float value, float min_val, float max_val);

/**
 * @brief Map float [-1,1] voltage demand to CAN command [-25000, 25000].
 */
int16_t voltage_to_can(float demand);

#endif // PID_CONTROLLER_H
