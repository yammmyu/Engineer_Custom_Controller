/**
 * @file pid.h
 * @brief Lightweight SerialPlot
 * @author yammmyu
 * @date 09-2025
 *
 * @copyright NUS Calibur Robotics (c) 2025
 */

#ifndef DVC_SERIALPLOT_H
#define DVC_SERIALPLOT_H

#include "stm32f4xx_hal.h"   // adjust to match your STM32 series
#include <stdint.h>
#include <stdarg.h>

/* ---------------- Configuration Macros ---------------- */

// Maximum number of simultaneous data pointers you can send (for plotting)
#define SERIALPLOT_MAX_DATA_PTRS  12
// Size of TX/RX buffers
#define SERIALPLOT_MAX_BUFFER     256
// Max length of variable names used in "var=value#" commands
#define SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH 32

/* ---------------- Data Types ---------------- */

// Supported data types for transmission
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

// Lightweight UART manager (keeps TX and RX buffers)
typedef struct {
    uint8_t Rx_Buffer[SERIALPLOT_MAX_BUFFER];
    uint8_t Tx_Buffer[SERIALPLOT_MAX_BUFFER];
} UART_Manage_t;

// Main serialplot object
typedef struct {
    UART_HandleTypeDef *huart;   // HAL UART handle
    UART_Manage_t *uart_mgr;     // Pointer to buffer manager

    uint8_t uart_rx_variable_num;  // Number of expected RX variables
    char **uart_rx_variable_list;  // List of variable names

    Serialplot_Data_Type_t tx_data_type; // Transmission data type
    uint8_t frame_header;                // Frame header identifier

    int8_t variable_index;   // Index of variable matched in dictionary
    double variable_value;   // Parsed value of the variable

    void *data[SERIALPLOT_MAX_DATA_PTRS];  // Pointers to data for TX
    uint8_t data_number;                   // Number of active data pointers
} Serialplot_t;

/* ---------------- Public API ---------------- */

/**
 * @brief Initialize a serialplot object
 */
void Serialplot_Init(Serialplot_t *obj,
                     UART_HandleTypeDef *huart,
                     UART_Manage_t *uart_mgr,
                     uint8_t rx_var_num,
                     char **rx_var_list,
                     Serialplot_Data_Type_t tx_type,
                     uint8_t frame_header);

/**
 * @brief Assign pointers to variables you want to send.
 * Example: Serialplot_SetData(&plot, 2, &var1, &var2);
 */
void Serialplot_SetData(Serialplot_t *obj, uint8_t num, ...);

/**
 * @brief Fill TX buffer with current data + header.
 * Does NOT call HAL_UART_Transmit (user decides how to send).
 */
void Serialplot_Output(Serialplot_t *obj);

/**
 * @brief Get index of last received variable.
 */
int8_t Serialplot_GetVariableIndex(Serialplot_t *obj);

/**
 * @brief Get value of last received variable.
 */
double Serialplot_GetVariableValue(Serialplot_t *obj);

/**
 * @brief Handle UART RX complete callback (parse variable=value#).
 */
void Serialplot_UART_RxCpltCallback(Serialplot_t *obj, uint8_t *rx_data);

/**
 * @brief Handle timer interrupt callback (update TX buffer).
 */
void Serialplot_TIM_PeriodElapsedCallback(Serialplot_t *obj);

#endif // DVC_SERIALPLOT_H
