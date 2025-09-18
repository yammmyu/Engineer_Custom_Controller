#include "dvc_motor.h"
#include <string.h>
#include "pid_params.h"
#include "pid.h"
#include <math.h>   // for lroundf if needed


/* ---------------- Internal Helpers ---------------- */

/**
 * @brief Allocate CAN Tx data pointer based on CAN ID.
 * @note Requires global buffers (e.g., CAN1_0x200_Tx_Data[]) defined elsewhere.
 */
static uint8_t* allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID id)
{
    // TODO: Map to actual global Tx buffers depending on hcan and id.
    // For now, return NULL as placeholder.
    return NULL;
}

/* ---------------- API Implementation ---------------- */

void Motor_Init(Motor_t *motor,
                CAN_HandleTypeDef *hcan,
                Enum_CAN_Motor_ID id,
                Enum_Control_Method method,
                float gearbox_rate,
                float torque_max,
                float omega_max,
                int32_t encoder_offset)
{
    memset(motor, 0, sizeof(Motor_t));

    motor->hcan = hcan;
    motor->CAN_ID = id;
    motor->Control_Method = method;
    motor->Gearbox_Rate = gearbox_rate;
    motor->Torque_Max = torque_max;
    motor->Omega_Max = omega_max;
    motor->Encoder_Offset = encoder_offset;

    motor->CAN_Tx_Data = allocate_tx_data(hcan, id);

    motor->Output_Max = 16384;   // example limit
    motor->Encoder_Num_Per_Round = 8192; // depends on motor
}

void Motor_Output(Motor_t *motor)
{
    if (motor->CAN_Tx_Data == NULL) return;

    motor->CAN_Tx_Data[0] = (int16_t)(motor->Out) >> 8;
    motor->CAN_Tx_Data[1] = (int16_t)(motor->Out);
}

/* ---------------- Getters ---------------- */

uint16_t Motor_Get_Output_Max(Motor_t *motor) { return motor->Output_Max; }
Enum_CAN_Motor_Status Motor_Get_Status(Motor_t *motor) { return motor->CAN_Motor_Status; }
float Motor_Get_Now_Angle(Motor_t *motor) { return motor->Now_Angle; }
float Motor_Get_Now_Omega(Motor_t *motor) { return motor->Now_Omega; }
float Motor_Get_Now_Torque(Motor_t *motor) { return motor->Now_Torque; }
uint8_t Motor_Get_Now_Temperature(Motor_t *motor) { return motor->Now_Temperature; }
Enum_Control_Method Motor_Get_Control_Method(Motor_t *motor) { return motor->Control_Method; }
float Motor_Get_Target_Angle(Motor_t *motor) { return motor->Target_Angle; }
float Motor_Get_Target_Omega(Motor_t *motor) { return motor->Target_Omega; }
float Motor_Get_Target_Torque(Motor_t *motor) { return motor->Target_Torque; }
float Motor_Get_Out(Motor_t *motor) { return motor->Out; }

/* ---------------- Setters ---------------- */

void Motor_Set_Control_Method(Motor_t *motor, Enum_Control_Method method) { motor->Control_Method = method; }
void Motor_Set_Target_Angle(Motor_t *motor, float angle) { motor->Target_Angle = angle; }
void Motor_Set_Target_Omega(Motor_t *motor, float omega) { motor->Target_Omega = omega; }
void Motor_Set_Target_Torque(Motor_t *motor, float torque) { motor->Target_Torque = torque; }
void Motor_Set_Out(Motor_t *motor, float out) { motor->Out = out; }

/* ---------------- Callbacks ---------------- */

void Motor_CAN_RxCpltCallback(Motor_t *motor, uint8_t *rx_data)
{
    int16_t delta_encoder;

    motor->Flag++;

    motor->Pre_Encoder = motor->Rx_Encoder;

    motor->Rx_Encoder   = (rx_data[0] << 8) | rx_data[1];
    int16_t rx_omega    = (rx_data[2] << 8) | rx_data[3];
    int16_t rx_torque   = (rx_data[4] << 8) | rx_data[5];
    uint8_t rx_temp     = rx_data[6];

    delta_encoder = motor->Rx_Encoder - motor->Pre_Encoder;
    if (delta_encoder < -4096) {
        motor->Total_Round++;
    } else if (delta_encoder > 4096) {
        motor->Total_Round--;
    }

    motor->Total_Encoder = motor->Total_Round * motor->Encoder_Num_Per_Round
                         + motor->Rx_Encoder + motor->Encoder_Offset;

    motor->Now_Angle = (float)motor->Total_Encoder / (float)motor->Encoder_Num_Per_Round
                     * 2.0f * PI / motor->Gearbox_Rate;
    motor->Now_Omega = (float)rx_omega * RPM_TO_RADPS / motor->Gearbox_Rate;
    motor->Now_Torque = rx_torque;
    motor->Now_Temperature = rx_temp;
}

void Motor_TIM_Alive_PeriodElapsedCallback(Motor_t *motor)
{
    if (motor->Flag == motor->Pre_Flag) {
        motor->CAN_Motor_Status = CAN_Motor_Status_DISABLE;
        // TODO: Reset PID integrals here if using PID module
    } else {
        motor->CAN_Motor_Status = CAN_Motor_Status_ENABLE;
    }
    motor->Pre_Flag = motor->Flag;
}

void Motor_TIM_PID_PeriodElapsedCallback(Motor_t *motor)
{
    switch (motor->Control_Method) {
    case Control_Method_OPENLOOP:
        // Open-loop: directly scale torque to output
        motor->Out = motor->Target_Torque / motor->Torque_Max * motor->Output_Max;
        break;

    case Control_Method_TORQUE:
        // Torque control: directly scale torque to output
        motor->Out = motor->Target_Torque / motor->Torque_Max * motor->Output_Max;
        break;

    case Control_Method_OMEGA:
        // Speed control: PID_Omega already clamps output inside pid_tick()
        pid_tick(&motor->PID_Omega);
        motor->Out = pid_get_out(&motor->PID_Omega);
        break;

    case Control_Method_ANGLE:
        // Double-loop control: angle PID outputs target speed, which feeds into speed PID
        pid_tick(&motor->PID_Angle);
        pid_set_target(&motor->PID_Omega, pid_get_out(&motor->PID_Angle));
        pid_tick(&motor->PID_Omega);
        motor->Out = pid_get_out(&motor->PID_Omega);
        break;

    default:
        motor->Out = 0.0f;
        break;
    }
}