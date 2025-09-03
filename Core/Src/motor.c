
#include "main.h"
#include "motor.h"
#ifndef PI
#define PI (3.1415927f)
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }


static motor_measure_t motor_chassis[6];

/* ------------------------------ 初始化（配置过滤器）------------------------------ */
void Enable_CAN1(void)
{
    CAN_FilterTypeDef CAN_Filter;
    CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter.FilterBank = 0;
    CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter.SlaveStartFilterBank = 0;
    CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
    CAN_Filter.FilterIdHigh = 0x0000;
    CAN_Filter.FilterIdLow = 0x0000;
    CAN_Filter.FilterMaskIdHigh= 0x0000;
    CAN_Filter.FilterMaskIdLow = 0x0000;
    if (HAL_CAN_ConfigFilter(&hcan1,&CAN_Filter)!= HAL_OK){
    	HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
        Error_Handler();
    }
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // CAN1 -> FIFO0
}


void Enable_CAN2(void)
{
    CAN_FilterTypeDef CAN_Filter;
    CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;   // Use FIFO1 for CAN2
    CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter.FilterBank = 14;   // Banks 0–13 reserved for CAN1, use 14+ for CAN2
    CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter.SlaveStartFilterBank = 14; // Splits filters between CAN1 and CAN2
    CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
    CAN_Filter.FilterIdHigh = 0x0000;
    CAN_Filter.FilterIdLow = 0x0000;
    CAN_Filter.FilterMaskIdHigh= 0x0000;
    CAN_Filter.FilterMaskIdLow = 0x0000;

    if (HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter) != HAL_OK){
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        Error_Handler();
    }

    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);  // CAN2 -> FIFO1
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
            .IDE = CAN_ID_STD,    // 标准帧
            .RTR = CAN_RTR_DATA,  // 数据帧
            .StdId = 0x200
    };
    uint32_t TxBox = CAN_TX_MAILBOX0;
    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxBox) != HAL_OK){
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);//错误处理
    }
}




/* ------------------------------------------ 接收函数 ------------------------------------------ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    if (hcan == &hcan1)
    {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)  // 获得接收到的数据头和数据
        {
            switch (RxHeader.StdId)
            {
            	case CAN_M2006_M1_ID: return 0;
            	case CAN_M2006_M2_ID: return 1;
            	case CAN_M2006_M3_ID: return 2;
            	case CAN_GM6020_M5_ID: return 3;
            	case CAN_GM6020_M6_ID: return 4;
            	case CAN_GM6020_M7_ID: return 5;
            	}
            }
        }
    }
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 再次使能FIFO0接收中断
}

