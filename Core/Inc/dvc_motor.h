/**
 * @file dvc_motor.h
 * @brief CAN motor configuration and operation (C version)
 * @author yammmyu
 * @date 2025-09-17
 */

#ifndef DVC_MOTOR_H
#define DVC_MOTOR_H

#include <stdint.h>
#include "alg_pid.h"

/* ---------------- Constants ---------------- */

#define PI              3.14159265358979323846f
#define RPM_TO_RADPS    (2.0f * PI / 60.0f)

/* ---------------- Enumerations ---------------- */

// Supported control methods
typedef enum {
    Control_Method_OPENLOOP = 0,
    Control_Method_TORQUE,
    Control_Method_OMEGA,
    Control_Method_ANGLE
} Enum_Control_Method;

// Motor status (alive or not)
typedef enum {
    CAN_Motor_Status_DISABLE = 0,
    CAN_Motor_Status_ENABLE
} Enum_CAN_Motor_Status;

// All CAN motor IDs
typedef enum {
    CAN_Motor_ID_0x201,
    CAN_Motor_ID_0x202,
    CAN_Motor_ID_0x203,
    CAN_Motor_ID_0x204,
    CAN_Motor_ID_0x205,
    CAN_Motor_ID_0x206,
    CAN_Motor_ID_0x207,
    CAN_Motor_ID_0x208,
    CAN_Motor_ID_0x209,
    CAN_Motor_ID_0x20A,
    CAN_Motor_ID_0x20B
} Enum_CAN_Motor_ID;

/* ---------------- Motor Object ---------------- */

/**
 * @brief Generic motor object for CAN-based motors.
 * Can be used for GM6020, C610, C620, etc.
 */
typedef struct {
    // Hardware references
    CAN_HandleTypeDef *hcan;   ///< HAL CAN handle
    uint8_t *CAN_Tx_Data;      ///< Pointer to CAN Tx buffer
    Enum_CAN_Motor_ID CAN_ID;  ///< Assigned CAN ID

    // Control configuration
    Enum_Control_Method Control_Method;
    float Gearbox_Rate;   ///< Gear reduction ratio (1 if no gearbox)
    float Torque_Max;     ///< Maximum torque (Nm)
    float Omega_Max;      ///< Maximum angular velocity (rad/s)
    int32_t Encoder_Offset; ///< Encoder offset value

    // Motor state (feedback)
    float Now_Angle;       ///< Current angle (rad)
    float Now_Omega;       ///< Current angular velocity (rad/s)
    float Now_Torque;      ///< Current torque (Nm or raw)
    uint8_t Now_Temperature; ///< Current temperature (°C)

    // Motor targets
    float Target_Angle;    ///< Desired angle (rad)
    float Target_Omega;    ///< Desired angular velocity (rad/s)
    float Target_Torque;   ///< Desired torque (Nm)
    float Out;             ///< Output command (scaled)

    // Encoder tracking
    uint16_t Rx_Encoder;   ///< Current encoder value
    uint16_t Pre_Encoder;  ///< Previous encoder value
    int32_t Total_Round;   ///< Number of full rotations
    int32_t Total_Encoder; ///< Absolute encoder count

    // Flags & status
    uint32_t Flag;         ///< Received message counter
    uint32_t Pre_Flag;     ///< Previous counter (for alive check)
    Enum_CAN_Motor_Status CAN_Motor_Status;

    // Limits
    uint16_t Output_Max;   ///< Maximum output value
    uint16_t Encoder_Num_Per_Round; ///< Encoder counts per revolution

    // PID controllers
    pid_t PID_Angle;   ///< Outer loop (angle control)
    pid_t PID_Omega;   ///< Inner loop (speed control)
    pid_t PID_Torque;  ///< Optional torque control
} Motor_t;

/* ---------------- API Functions ---------------- */

// Initialization
void Motor_Init(Motor_t *motor,
                CAN_HandleTypeDef *hcan,
                Enum_CAN_Motor_ID id,
                Enum_Control_Method method,
                float gearbox_rate,
                float torque_max,
                float omega_max,
                int32_t encoder_offset);

// CAN Tx buffer output
void Motor_Output(Motor_t *motor);

// Getters
uint16_t Motor_Get_Output_Max(Motor_t *motor);
Enum_CAN_Motor_Status Motor_Get_Status(Motor_t *motor);
float Motor_Get_Now_Angle(Motor_t *motor);
float Motor_Get_Now_Omega(Motor_t *motor);
float Motor_Get_Now_Torque(Motor_t *motor);
uint8_t Motor_Get_Now_Temperature(Motor_t *motor);
Enum_Control_Method Motor_Get_Control_Method(Motor_t *motor);
float Motor_Get_Target_Angle(Motor_t *motor);
float Motor_Get_Target_Omega(Motor_t *motor);
float Motor_Get_Target_Torque(Motor_t *motor);
float Motor_Get_Out(Motor_t *motor);

// Setters
void Motor_Set_Control_Method(Motor_t *motor, Enum_Control_Method method);
void Motor_Set_Target_Angle(Motor_t *motor, float angle);
void Motor_Set_Target_Omega(Motor_t *motor, float omega);
void Motor_Set_Target_Torque(Motor_t *motor, float torque);
void Motor_Set_Out(Motor_t *motor, float out);

// Callbacks
void Motor_CAN_RxCpltCallback(Motor_t *motor, uint8_t *rx_data);
void Motor_TIM_Alive_PeriodElapsedCallback(Motor_t *motor);
void Motor_TIM_PID_PeriodElapsedCallback(Motor_t *motor);

#endif // DVC_MOTOR_H
