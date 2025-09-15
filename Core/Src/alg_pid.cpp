/**
 * @file alg_pid.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief PID算法
 * @version 0.1
 * @date 2022-05-08
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief PID initilaization
 *
 * @param __K_P P
 * @param __K_I I
 * @param __K_D D
 * @param __K_F Feedforward
 * @param __I_Out_Max Intergral Error Max
 * @param __Out_Max Output Max
 * @param __D_T Timer Period
 */
void Class_PID::Init(float __K_P, float __K_I, float __K_D, float __K_F, float __I_Out_Max, float __Out_Max, float __D_T, float __Dead_Zone, float __I_Variable_Speed_A, float __I_Variable_Speed_B, float __I_Separate_Threshold, Enum_PID_D_First __D_First)
{
    K_P = __K_P;
    K_I = __K_I;
    K_D = __K_D;
    K_F = __K_F;
    I_Out_Max = __I_Out_Max;
    Out_Max = __Out_Max;
    D_T = __D_T;
    Dead_Zone = __Dead_Zone;
    I_Variable_Speed_A = __I_Variable_Speed_A;
    I_Variable_Speed_B = __I_Variable_Speed_B;
    I_Separate_Threshold = __I_Separate_Threshold;
    D_First = __D_First;
}

/**
 * @brief Get Integral Error
 *
 * @return float Intergral Error
 */
float Class_PID::Get_Integral_Error()
{
    return (Integral_Error);
}

/**
 * @brief Get Output
 *
 * @return float Output
 */
float Class_PID::Get_Out()
{
    return (Out);
}

/**
 * @brief Set P Value
 *
 * @param __K_P K_P
 */
void Class_PID::Set_K_P(float __K_P)
{
    K_P = __K_P;
}

/**
 * @brief Set I Value
 *
 * @param __K_I K_I
 */
void Class_PID::Set_K_I(float __K_I)
{
    K_I = __K_I;
}

/**
 * @brief Set D Value
 *
 * @param __K_D K_D
 */
void Class_PID::Set_K_D(float __K_D)
{
    K_D = __K_D;
}

/**
 * @brief Set Feedforward
 *
 * @param __K_F K_F
 */
void Class_PID::Set_K_F(float __K_F)
{
    K_F = __K_F;
}

/**
 * @brief Set Integral Error Max, Set to 0 to disable
 *
 * @param __I_Out_Max Integral Error Max, Set to 0 to disable
 */
void Class_PID::Set_I_Out_Max(float __I_Out_Max)
{
    I_Out_Max = __I_Out_Max;
}

/**
 * @brief Set Output Max, set to 0 to disable
 *
 * @param __Out_Max Set Output Max, Set to 0 to disable
 */
void Class_PID::Set_Out_Max(float __Out_Max)
{
    Out_Max = __Out_Max;
}

/**
 * @brief Set Variable Integral Error Change Begin Threshold, set to 0 to disable
 *
 * @param __I_Variable_Speed_A Variable Error Change Begin Threshold
 */
void Class_PID::Set_I_Variable_Speed_A(float __I_Variable_Speed_A)
{
    I_Variable_Speed_A = __I_Variable_Speed_A;
}

/**
 * @brief Set Variable Intergral Error Change End / Seperation Threshold, set to 0 to disable
 *
 * @param __I_Variable_Speed_B Variable Intergral Error Change End / Seperation Threshold, set to 0 to disable
 */
void Class_PID::Set_I_Variable_Speed_B(float __I_Variable_Speed_B)
{
    I_Variable_Speed_B = __I_Variable_Speed_B;
}

/**
 * @brief Set Integral Seperation Threshold
 *
 * @param __I_Separate_Threshold Set Intrgral Seperation Threshold, Must be positive integer, set to 0 to disable
 */
void Class_PID::Set_I_Separate_Threshold(float __I_Separate_Threshold)
{
    I_Separate_Threshold = __I_Separate_Threshold;
}

/**
 * @brief Set Target Value
 *
 * @param __Target Target Value
 */
void Class_PID::Set_Target(float __Target)
{
    Target = __Target;
}

/**
 * @brief Set Now Value
 *
 * @param __Now Now Value
 */
void Class_PID::Set_Now(float __Now)
{
    Now = __Now;
}

/**
 * @brief Set Integral Error, used for clearing Integral Error
 *
 * @param __Set_Integral_Error Integral Error
 */
void Class_PID::Set_Integral_Error(float __Integral_Error)
{
    Integral_Error = __Integral_Error;
}

/**
 * @brief PID Calculation
 *
 * @return float Output
 */
void Class_PID::TIM_Adjust_PeriodElapsedCallback()
{
    // P
    float p_out = 0.0f;
    // I
    float i_out = 0.0f;
    // D
    float d_out = 0.0f;
    // Feedforward
    float f_out = 0.0f;
    // Error
    float error;
    // Absolute Error
    float abs_error;
    // f(Integral_Error) linear function slope
    float speed_ratio;

    error = Target - Now;
    abs_error = Math_Abs(error);

    //Determine if is Deadzone
    if (abs_error < Dead_Zone)
    {
        Target = Now;
        error = 0.0f;
        abs_error = 0.0f;
    }

    //P Calc

    p_out = K_P * error;

    //I Calc

    if (I_Variable_Speed_A == 0.0f && I_Variable_Speed_B == 0.0f)
    {
        //Not Variable Speed Integral
        speed_ratio = 1.0f;
    }
    else
    {
        if (abs_error <= I_Variable_Speed_A)
        {
            // Full gain zone
            speed_ratio = 1.0f;
        }
        else if (abs_error < I_Variable_Speed_B)
        {
            // Fade-out zone (linear)
            speed_ratio = (I_Variable_Speed_B - abs_error) / (I_Variable_Speed_B - I_Variable_Speed_A);
        }
        else
        {
            // No gain zone
            speed_ratio = 0.0f;
        }
    }
    }


    //Integral Error Max
    if (I_Out_Max != 0.0f)
    {
        Math_Constrain(&Integral_Error, -I_Out_Max / K_I, I_Out_Max / K_I);
    }
    if (I_Separate_Threshold == 0.0f)
    {
        //No Integral Seperation
        Integral_Error += speed_ratio * D_T * error;
        i_out = K_I * Integral_Error;
    }
    else
    {
        //Integral Seperation Operation
        if (abs_error < I_Separate_Threshold)
        {
            Integral_Error += speed_ratio * D_T * error;
            i_out = K_I * Integral_Error;
        }
        else
        {
            Integral_Error = 0.0f;
            i_out = 0.0f;
        }
    }

    //K_D Calc

    if (D_First == PID_D_First_DISABLE)
    {
        //No Derivative First
        d_out = K_D * (error - Pre_Error) / D_T;
    }
    else
    {
        //Derivative First Operation
        d_out = K_D * (Out - Pre_Out) / D_T;
    }

    //Calculate Feedforward

    f_out = (Target - Pre_Target) * K_F;

    //Calculate Final Ouput

    Out = p_out + i_out + d_out + f_out;
    
    //Output Max
    if (Out_Max != 0.0f)
    {
        Math_Constrain(&Out, -Out_Max, Out_Max);
    }

    //Finishing up
    Pre_Now = Now;
    Pre_Target = Target;
    Pre_Out = Out;
    Pre_Error = error;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
