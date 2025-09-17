/**
 * @file pid.h
 * @brief Lightweight PID controller for Engineer Custom Controller
 * @author yammmyu
 * @date 09-2025
 *
 * @copyright Calibur Robotics (c) 2025
 */
#ifndef DVC_SERIALPLOT_H
#define DVC_SERIALPLOT_H

#include "stm32f4xx_hal.h"   // adjust series for your MCU
#include <stdint.h>
#include <stdarg.h>

#define SERIALPLOT_MAX_DATA_PTRS  12
#define SERIALPLOT_MAX_BUFFER     256
#define SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH 32

// Supported data types
typedef enum {
    Serialplot_Data_Type_UINT8,
    Serialplot_Data_Type_INT8,
    Serialplot_Data_Type_UINT16,
    Serialplot_Data_Type_INT16,
    Serialplot_Data_Type_UINT32,
    Serialplot_Data_Type_INT32,
    Serialplot_Data_Type_FLOAT,
    Serialplot_Data_Type_DOUBLE
} Serialplot_Data_Type_t;

// UART manager struct (simplified version)
typedef struct {
    uint8_t Rx_Buffer[SERIALPLOT_MAX_BUFFER];
    uint8_t Tx_Buffer[SERIALPLOT_MAX_BUFFER];
} UART_Manage_t;

// Main serialplot struct
typedef struct {
    UART_HandleTypeDef *huart;
    UART_Manage_t *uart_mgr;

    uint8_t uart_rx_variable_num;
    char **uart_rx_variable_list;

    Serialplot_Data_Type_t tx_data_type;
    uint8_t frame_header;

    int8_t variable_index;
    double variable_value;

    void *data[SERIALPLOT_MAX_DATA_PTRS];
    uint8_t data_number;
} Serialplot_t;

/* API */
void Serialplot_Init(Serialplot_t *obj,
                     UART_HandleTypeDef *huart,
                     UART_Manage_t *uart_mgr,
                     uint8_t rx_var_num,
                     char **rx_var_list,
                     Serialplot_Data_Type_t tx_type,
                     uint8_t frame_header);

void Serialplot_SetData(Serialplot_t *obj, uint8_t num, ...);
void Serialplot_Output(Serialplot_t *obj);

int8_t Serialplot_GetVariableIndex(Serialplot_t *obj);
double Serialplot_GetVariableValue(Serialplot_t *obj);

void Serialplot_UART_RxCpltCallback(Serialplot_t *obj, uint8_t *rx_data);
void Serialplot_TIM_PeriodElapsedCallback(Serialplot_t *obj);

#endif // DVC_SERIALPLOT_H
