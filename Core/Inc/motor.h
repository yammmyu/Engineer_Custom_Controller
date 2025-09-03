
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

//CAN send and receive ID
typedef enum
{
	CAN_M2006_M1_ID = 0x201,
	CAN_M2006_M2_ID = 0x202,
	CAN_M2006_M3_ID = 0x203,


	CAN_GM6020_M5_ID = 0x205,
	CAN_GM6020_M6_ID = 0x206,
	CAN_GM6020_M7_ID = 0x207,

} can_msg_id_e;


//RM motor data
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperature;
	int16_t last_ecd;
} motor_measure_t;


#endif //MOTOR_H

