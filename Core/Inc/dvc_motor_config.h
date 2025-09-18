/**
 * @file motor_init.h
 * @brief Motor and PID initialization for all motors
 */

#ifndef MOTOR_INIT_H
#define MOTOR_INIT_H

#include "dvc_motor.h"
#include "cfg_pid_params.h"
#include "stm32f4xx_hal.h"

// Number of motors in your system
#define MOTOR_COUNT 6

// Declare global motor objects
extern Motor_t motors[MOTOR_COUNT];

/**
 * @brief Initialize all motors and attach their PID parameters
 * 
 * This function should be called after CAN buses are initialized.
 */
void Motors_Init(void);

#endif // MOTOR_INIT_H
