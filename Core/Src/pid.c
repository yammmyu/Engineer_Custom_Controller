// PID.c
#include "PID.h"
#include "motor.h"
#include <math.h>  // NEW: For fmod in angle error calculation

// Global safety flag (NEW)
static bool system_in_safe_mode = false;

MotorPID motor_pids[NUM_MOTORS];

void PIDController_Init(PIDController *pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->outMin = out_min;
    pid->outMax = out_max;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    float proportional = pid->Kp * error;

    // Integral with clamp
    pid->integral += pid->Ki * error * dt;
    if (pid->integral > pid->outMax) pid->integral = pid->outMax;
    if (pid->integral < pid->outMin) pid->integral = pid->outMin;

    // Derivative term with simple clamp to avoid spikes (NEW)
    float derivative = 0.0f;
    if (dt > 1e-6f) {
        derivative = pid->Kd * (error - pid->prev_error) / dt;
        if (derivative > pid->outMax) derivative = pid->outMax;
        if (derivative < pid->outMin) derivative = pid->outMin;
    }

    float output = proportional + pid->integral + derivative;
    if (output > pid->outMax) output = pid->outMax;
    if (output < pid->outMin) output = pid->outMin;

    pid->prev_error = error;
    return output;
}

void MotorPID_Init(void) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_pids[i].motor = &all_motors[i];
        motor_pids[i].target_position = 0.0f;
        motor_pids[i].is_gm6020 = (i >= 3);
        motor_pids[i].active = false;  // Motor starts as inactive

        // Position PID
        PIDController_Init(&motor_pids[i].pos_pid, 2.0f, 0.01f, 0.0f, -5000.0f, 5000.0f);

        // Velocity PID
        if (motor_pids[i].is_gm6020) {
            PIDController_Init(&motor_pids[i].vel_pid, 1.0f, 0.05f, 0.05f, -20000.0f, 20000.0f);
        } else {
            PIDController_Init(&motor_pids[i].vel_pid, 1.0f, 0.05f, 0.05f, -10000.0f, 10000.0f);
        }
    }

    Disable_All_Motors();  // NEW: Ensure all motors are off at init
}

void Set_Motor_Position(int index, float position_deg) {
    if (index < 0 || index >= NUM_MOTORS) return;

    // Clamp to safe range (NEW)
    if (position_deg > SAFE_MAX_ANGLE) position_deg = SAFE_MAX_ANGLE;
    if (position_deg < SAFE_MIN_ANGLE) position_deg = SAFE_MIN_ANGLE;

    motor_pids[index].target_position = position_deg;
    motor_pids[index].active = true;
}

void Disable_Motor(int index) {
    if (index < 0 || index >= NUM_MOTORS) return;
    motor_pids[index].active = false;
    motor_pids[index].pos_pid.integral = 0.0f;
    motor_pids[index].vel_pid.integral = 0.0f;
}

void Disable_All_Motors(void) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_pids[i].active = false;
        motor_pids[i].pos_pid.integral = 0.0f;
        motor_pids[i].vel_pid.integral = 0.0f;
    }
}

void Set_Safe_Mode(bool enable) {  // NEW: external kill switch
    system_in_safe_mode = enable;
    if (enable) {
        Disable_All_Motors();
        Set_M2006_Current(0, 0, 0);
        Set_GM6020_Current(0, 0, 0);
    }
}

void Motor_Control_Update(void) {
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    // Skip on first run (NEW)
    if (last_tick == 0) {
        last_tick = now;
        Set_M2006_Current(0, 0, 0);
        Set_GM6020_Current(0, 0, 0);
        return;
    }

    float dt = (now - last_tick) / 1000.0f;
    last_tick = now;
    if (dt <= 0.0f) dt = 0.001f;

    // Global safety check (NEW)
    if (system_in_safe_mode) {
        Set_M2006_Current(0, 0, 0);
        Set_GM6020_Current(0, 0, 0);
        return;
    }

    int16_t currents[NUM_MOTORS] = {0};

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!motor_pids[i].active) {
            currents[i] = 0;  // inactive motors always zero
            continue;
        }

        // Compute position error with shortest path (NEW: to handle wrapping)
        float pos_error = motor_pids[i].target_position - motor_pids[i].motor->angle_deg;
        pos_error = fmodf(pos_error + 180.0f, 360.0f) - 180.0f;

        float vel_set = PIDController_Update(
            &motor_pids[i].pos_pid,
            motor_pids[i].target_position,
            motor_pids[i].motor->angle_deg,
            dt
        );

        float current = PIDController_Update(
            &motor_pids[i].vel_pid,
            vel_set,
            (float)motor_pids[i].motor->speed_rpm,
            dt
        );

        // Explicit cast with bounds check (NEW: though clamps should prevent overflow)
        if (current > 32767.0f) current = 32767.0f;
        if (current < -32768.0f) current = -32768.0f;
        currents[i] = (int16_t)current;
    }

    // Send to M2006 (indices 0,1,2)
    Set_M2006_Current(currents[0], currents[1], currents[2]);

    // Send to GM6020 (indices 3,4,5)
    Set_GM6020_Current(currents[3], currents[4], currents[5]);
}
