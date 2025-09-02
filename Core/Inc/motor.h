
//
// Created by 29851 on 2024/6/29.
//

#ifndef MOTOR_H
#define MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/*函数声名*/
void Enable_CAN1(void);
void Enable_CAN2(void);
void Set_GM6020_Current(int16_t q1, int16_t q2, int16_t q3);
void Set_M2006_Current(int16_t q1, int16_t q2, int16_t q3);



#endif //PIKA_PIKA_MOTOR_H

