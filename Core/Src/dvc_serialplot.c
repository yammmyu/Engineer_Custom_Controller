/**
 * @file pid.h
 * @brief Lightweight PID controller for Engineer Custom Controller
 * @author yammmyu
 * @date 09-2025
 *
 * @copyright Calibur Robotics (c) 2025
 */

#include "dvc_serialplot.h"
#include <string.h>
#include <math.h>

/* ----------------- Internal helpers ------------------ */

static uint8_t Serialplot_JudgeVariableName(Serialplot_t *obj)
{
    char tmp_variable_name[SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH];
    int flag = 0;

    // copy until '='
    while (obj->uart_mgr->Rx_Buffer[flag] != '=' &&
           obj->uart_mgr->Rx_Buffer[flag] != '\0')
    {
        tmp_variable_name[flag] = obj->uart_mgr->Rx_Buffer[flag];
        flag++;
    }
    tmp_variable_name[flag] = '\0';

    for (int i = 0; i < obj->uart_rx_variable_num; i++) {
        if (strcmp(tmp_variable_name, obj->uart_rx_variable_list[i]) == 0) {
            obj->variable_index = i;
            return (uint8_t)(flag + 1);
        }
    }

    obj->variable_index = -1;
    return (uint8_t)(flag + 1);
}

static void Serialplot_JudgeVariableValue(Serialplot_t *obj, int flag)
{
    int tmp_dot_flag = 0, tmp_sign = 1, i;
    obj->variable_value = 0.0;

    if (obj->variable_index == -1) {
        return; // unknown variable, skip
    }

    if (obj->uart_mgr->Rx_Buffer[flag] == '-') {
        tmp_sign = -1;
        flag++;
    }

    for (i = flag;
         obj->uart_mgr->Rx_Buffer[i] != '#' &&
         obj->uart_mgr->Rx_Buffer[i] != '\0';
         i++)
    {
        if (obj->uart_mgr->Rx_Buffer[i] == '.') {
            tmp_dot_flag = i;
        } else {
            obj->variable_value =
                obj->variable_value * 10.0 +
                (obj->uart_mgr->Rx_Buffer[i] - '0');
        }
    }

    if (tmp_dot_flag != 0) {
        obj->variable_value /= pow(10.0, (double)(i - tmp_dot_flag - 1));
    }

    obj->variable_value *= tmp_sign;
}

/* ----------------- Public API ------------------ */

void Serialplot_Init(Serialplot_t *obj,
                     UART_HandleTypeDef *huart,
                     UART_Manage_t *uart_mgr,
                     uint8_t rx_var_num,
                     char **rx_var_list,
                     Serialplot_Data_Type_t tx_type,
                     uint8_t frame_header)
{
    obj->huart = huart;
    obj->uart_mgr = uart_mgr;
    obj->uart_rx_variable_num = rx_var_num;
    obj->uart_rx_variable_list = rx_var_list;
    obj->tx_data_type = tx_type;
    obj->frame_header = frame_header;

    obj->variable_index = -1;
    obj->variable_value = 0.0;
    obj->data_number = 0;

    memset(obj->uart_mgr->Tx_Buffer, 0, SERIALPLOT_MAX_BUFFER);
    obj->uart_mgr->Tx_Buffer[0] = frame_header;
}

void Serialplot_SetData(Serialplot_t *obj, uint8_t num, ...)
{
    va_list args;
    va_start(args, num);

    if (num > SERIALPLOT_MAX_DATA_PTRS) num = SERIALPLOT_MAX_DATA_PTRS;
    for (int i = 0; i < num; i++) {
        obj->data[i] = va_arg(args, void *);
    }

    va_end(args);
    obj->data_number = num;
}

void Serialplot_Output(Serialplot_t *obj)
{
    memset(obj->uart_mgr->Tx_Buffer, 0, SERIALPLOT_MAX_BUFFER);
    obj->uart_mgr->Tx_Buffer[0] = obj->frame_header;

    if (obj->tx_data_type == Serialplot_Data_Type_UINT8 ||
        obj->tx_data_type == Serialplot_Data_Type_INT8) {
        for (int i = 0; i < obj->data_number; i++) {
            memcpy(obj->uart_mgr->Tx_Buffer + i * sizeof(uint8_t) + 1,
                   obj->data[i], sizeof(uint8_t));
        }
    }
    else if (obj->tx_data_type == Serialplot_Data_Type_UINT16 ||
             obj->tx_data_type == Serialplot_Data_Type_INT16) {
        for (int i = 0; i < obj->data_number; i++) {
            memcpy(obj->uart_mgr->Tx_Buffer + i * sizeof(uint16_t) + 1,
                   obj->data[i], sizeof(uint16_t));
        }
    }
    else if (obj->tx_data_type == Serialplot_Data_Type_UINT32 ||
             obj->tx_data_type == Serialplot_Data_Type_INT32 ||
             obj->tx_data_type == Serialplot_Data_Type_FLOAT) {
        for (int i = 0; i < obj->data_number; i++) {
            memcpy(obj->uart_mgr->Tx_Buffer + i * sizeof(uint32_t) + 1,
                   obj->data[i], sizeof(uint32_t));
        }
    }
    else if (obj->tx_data_type == Serialplot_Data_Type_DOUBLE) {
        for (int i = 0; i < obj->data_number; i++) {
            memcpy(obj->uart_mgr->Tx_Buffer + i * sizeof(uint64_t) + 1,
                   obj->data[i], sizeof(uint64_t));
        }
    }
}

int8_t Serialplot_GetVariableIndex(Serialplot_t *obj)
{
    return obj->variable_index;
}

double Serialplot_GetVariableValue(Serialplot_t *obj)
{
    return obj->variable_value;
}

void Serialplot_UART_RxCpltCallback(Serialplot_t *obj, uint8_t *rx_data)
{
    int flag = Serialplot_JudgeVariableName(obj);
    Serialplot_JudgeVariableValue(obj, flag);
}

void Serialplot_TIM_PeriodElapsedCallback(Serialplot_t *obj)
{
    Serialplot_Output(obj);
}
