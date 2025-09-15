/**
 * @file alg_pid.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief PID算法
 * @version 0.1
 * @date 2022-05-03
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef ALG_PID_H
#define ALG_PID_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Derivate First Status 微分先行
 *
 */
enum Enum_PID_D_First
{
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
};

/**
 * @brief Reusable, PID Algorithum 
 *
 */
class Class_PID
{
public:
    void Init(float __K_P, float __K_I, float __K_D, float __K_F = 0.0f, float __I_Out_Max = 0.0f, float __Out_Max = 0.0f, float __D_T = 0.001f, float __Dead_Zone = 0.0f, float __I_Variable_Speed_A = 0.0f, float __I_Variable_Speed_B = 0.0f, float __I_Separate_Threshold = 0.0f, Enum_PID_D_First __D_First = PID_D_First_DISABLE);

    float Get_Integral_Error();
    float Get_Out();

    void Set_K_P(float __K_P);
    void Set_K_I(float __K_I);
    void Set_K_D(float __K_D);
    void Set_K_F(float __K_F); // Feedforward Control Variable, 前馈
    void Set_I_Out_Max(float __I_Out_Max); // Max Intergral Error
    void Set_Out_Max(float __Out_Max); // Max Output
    void Set_I_Variable_Speed_A(float __Variable_Speed_I_A); // Integral Error Change Begin Threshold 
    void Set_I_Variable_Speed_B(float __Variable_Speed_I_B); // Integral Error Change end / Seperation Threshold 
                                                             // Variable_Speed_Integral_Error = k * Integral_Error
                                                             // Integral_Error < a, k = 1 | a < Integral_Error < b, k = f(Integral_Error)| k = f(integral_error)b, k = 0 |
    void Set_I_Separate_Threshold(float __I_Separate_Threshold); // Intergral Seperating Threshold
    void Set_Target(float __Target);
    void Set_Now(float __Now);
    void Set_Integral_Error(float __Integral_Error); // Used to clear integral error in case of error accumulation when referee system cuts power.

    void TIM_Adjust_PeriodElapsedCallback(); // Timer

protected:
    //Constants Related to Initialization

    // PID timer period, s
    float D_T;
    // Deadzone, Disables PID output when error is within this range
    float Dead_Zone;
    // Derivative First
    Enum_PID_D_First D_First;

    //Constants

    //Internal Vairables

    //Previous Recorded Value
    float Pre_Now = 0.0f;
    //Previous Target Value
    float Pre_Target = 0.0f;
    //Previous Output
    float Pre_Out = 0.0f;
    //Previous Error
    float Pre_Error = 0.0f;

    //Read Variables

    //Output
    float Out = 0.0f;

    //Write Variables

    // P
    float K_P = 0.0f;
    // I
    float K_I = 0.0f;
    // D
    float K_D = 0.0f;
    // Feedforward
    float K_F = 0.0f;

    //Integral Error Limit, set to 0 to disable
    float I_Out_Max = 0;
    //Output Lomit, set to 0 to disable
    float Out_Max = 0;

    //Variable Speed Integral Upper, set to 0 to disable
    float I_Variable_Speed_A = 0.0f;
    //Variabke Speed Intergal Lower, set to 0 to disable
    float I_Variable_Speed_B = 0.0f;

    //Integral Seperatation Threshold，Must be a positive integer, set to 0 to disable
    float I_Separate_Threshold = 0.0f;

    //Target Value
    float Target = 0.0f;
    //Current Value
    float Now = 0.0f;

    //Read Write Variables

    //Integral Error Value
    float Integral_Error = 0.0f;

    //Internal Functions
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
