
//
// Created by 29851 on 2024/6/29.
//

#ifndef PIKA_PIKA_MOTOR_H
#define PIKA_PIKA_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/*函数声名*/
void Enable_CAN1(void);
void Set_Motor_Current(int16_t q1, int16_t q2, int16_t q3, int16_t q4);



#endif //PIKA_PIKA_MOTOR_H

