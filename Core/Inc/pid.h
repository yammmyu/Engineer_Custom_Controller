// PID.h
#ifndef PID_H
#define PID_H

#include "motor.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>  // For bool type

#define NUM_MOTORS 6
#define SAFE_MIN_ANGLE -180.0f
#define SAFE_MAX_ANGLE  180.0f

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float outMin;
    float outMax;
} PIDController;

typedef struct {
    PIDController pos_pid;
    PIDController vel_pid;
    float target_position;  // in degrees
    motor_measure_t *motor;
    int is_gm6020;  // 1 for GM6020, 0 for M2006
    bool active;    // NEW: Added for safety features
} MotorPID;

extern MotorPID motor_pids[NUM_MOTORS];

void PIDController_Init(PIDController *pid, float kp, float ki, float kd, float out_min, float out_max);
float PIDController_Update(PIDController *pid, float setpoint, float measurement, float dt);

void MotorPID_Init(void);
void Set_Motor_Position(int index, float position_deg);
void Motor_Control_Update(void);
void Disable_Motor(int index);
void Disable_All_Motors(void);
void Set_Safe_Mode(bool enable);

#endif // PID_H
