
#include "main.h"
#include "motor.h"
#ifndef PI
#define PI (3.1415927f)
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

motor_measure_t all_motors[6];

//static in-line function to decode the CAN message feedback
static inline void get_motor_measure(motor_measure_t *ptr, const uint8_t *data)
    {
        ptr ->last_ecd			= ptr->ecd;
        ptr ->ecd				= (uint16_t)((data[0] << 8) | data[1]);
        ptr ->speed_rpm 		= (uint16_t)((data)[2] << 8 | (data)[3]);
        ptr ->given_current		= (uint16_t)((data)[4] << 8 | (data)[5]);
        ptr ->temperature 		= (data)[6];
    }
/* ------------------------------ Initialization of CAN, filter Setup）------------------------------ */
void Enable_CAN2(void)
{
    CAN_FilterTypeDef CAN_Filter;

    CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter.FilterBank = 14;             // start filters for CAN2 here
    CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter.SlaveStartFilterBank = 14;   // 0–13 for CAN1, 14–27 for CAN2
    CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
    CAN_Filter.FilterIdHigh = 0x0000;
    CAN_Filter.FilterIdLow = 0x0000;
    CAN_Filter.FilterMaskIdHigh = 0x0000;   // accept all IDs
    CAN_Filter.FilterMaskIdLow = 0x0000;

    if (HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter) != HAL_OK){
        Error_Handler();
    }
    // 👇 Add this check here
    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    	Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    	Error_Handler();
    }

    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


/* ------------------------------ 发送函数 ------------------------------ */
void Set_GM6020_Current(int16_t q1, int16_t q2, int16_t q3)
{
    uint8_t TxData[8];
    TxData[0] = (uint8_t)(q1>>8);
    TxData[1] = (uint8_t)q1;
    TxData[2] = (uint8_t)(q2>>8);
    TxData[3] = (uint8_t)q2;
    TxData[4] = (uint8_t)(q3>>8);
    TxData[5] = (uint8_t)q3;
    TxData[6] = (uint8_t)0;
    TxData[7] = (uint8_t)0;
    CAN_TxHeaderTypeDef TxHeader = {
            .DLC = 8,
            .IDE = CAN_ID_STD,    // 标准帧
            .RTR = CAN_RTR_DATA,  // 数据帧
            .StdId = 0x2ff
    };
    uint32_t TxBox = CAN_TX_MAILBOX0;
    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxBox) != HAL_OK){
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);//错误处理
    }
}


void Set_M2006_Current(int16_t q1, int16_t q2, int16_t q3)
{
    uint8_t TxData[8];
    TxData[0] = (uint8_t)(q1>>8);
    TxData[1] = (uint8_t)q1;
    TxData[2] = (uint8_t)(q2>>8);
    TxData[3] = (uint8_t)q2;
    TxData[4] = (uint8_t)(q3>>8);
    TxData[5] = (uint8_t)q3;
    TxData[6] = (uint8_t)0;
    TxData[7] = (uint8_t)0;
    CAN_TxHeaderTypeDef TxHeader = {
            .DLC = 8,
            .IDE = CAN_ID_STD,    //standard frame
            .RTR = CAN_RTR_DATA,  //data frame
            .StdId = 0x200
    };
    uint32_t TxBox = CAN_TX_MAILBOX0;
    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxBox) != HAL_OK){
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);//错误处理
    }
}

//maps the CAN feedback ID with the motor ID
/*
 * M2060 0x200 + driver ID
 * 0x201 --> 1
 * 0x202 --> 2
 * 0x203 --> 3
 *
 * GM6020 0x204 + driver ID
 * 0x205 --> 4
 * 0x206 --> 5
 * 0x207 --> 6
 * */
static int8_t motor_index_from_id(uint16_t id)
{
	switch (id)
	{
		case CAN_M2006_M1_ID: return 0;
		case CAN_M2006_M2_ID: return 1;
		case CAN_M2006_M3_ID: return 2;
		case CAN_GM6020_M5_ID: return 3;
		case CAN_GM6020_M6_ID: return 4;
		case CAN_GM6020_M7_ID: return 5;

		default: return -1; //safety, filters any noise
	}
}


//Callback function to receive data
/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1 || hcan == &hcan2)
	{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];

		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
		{
			int8_t idx = motor_index_from_id(rx_header.StdId);
			if (idx >= 0)
			{
				get_motor_measure(&all_motors[idx], rx_data);

				all_motors[idx].angle_deg = (all_motors[idx].ecd / 8192.0f) * 360.0f;
			}

		}
	}

	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);  // 👈 Debug indicator

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);  // 👈 Debug indicator

        int8_t idx = motor_index_from_id(rx_header.StdId);
        if (idx >= 0)
        {
            get_motor_measure(&all_motors[idx], rx_data);
            all_motors[idx].angle_deg = (all_motors[idx].ecd / 8192.0f) * 360.0f;
        }
    }
}
