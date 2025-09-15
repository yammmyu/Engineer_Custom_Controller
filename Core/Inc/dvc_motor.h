/**
 * @file dvc_motor.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief CAN电机配置与操作
 * @version 0.1
 * @date 2022-08-03
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DVC_MOTOR_H
#define DVC_MOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "drv_can.h"

/* Exported macros -----------------------------------------------------------*/

// RPM换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Motor Status
 *
 */
enum Enum_CAN_Motor_Status
{
    CAN_Motor_Status_DISABLE = 0,
    CAN_Motor_Status_ENABLE,
};

/**
 * @brief Name List of all the possible motor CAN IDs
 *
 */
enum Enum_CAN_Motor_ID //Also Acts as a type control, to make sure the correct type is inputed
{
    CAN_Motor_ID_UNDEFINED = 0,
    CAN_Motor_ID_0x201,
    CAN_Motor_ID_0x202,
    CAN_Motor_ID_0x203,
    CAN_Motor_ID_0x204,
    CAN_Motor_ID_0x205,
    CAN_Motor_ID_0x206,
    CAN_Motor_ID_0x207,
    CAN_Motor_ID_0x208,
    CAN_Motor_ID_0x209,
    CAN_Motor_ID_0x20A,
    CAN_Motor_ID_0x20B,
};

/**
 * @brief occupation status of a CAN motor ID
 *
 */
enum Enum_CAN_Motor_ID_Status
{
    CAN_Motor_ID_Status_FREE = 0,
    CAN_Motor_ID_Status_ALLOCATED,
};

/**
 * @brief Motor Control method
 *
 */
enum Enum_Control_Method
{
    Control_Method_OPENLOOP = 0,
    Control_Method_TORQUE,
    Control_Method_OMEGA,
    Control_Method_ANGLE,
};

/**
 * @brief GM6020 brush-less Motor, MCU controlling Voltage output
 *
 */
class Class_Motor_GM6020
{
public:
    // PID Angle Loop
    Class_PID PID_Angle;
    // PID AngularVelocity Loop
    Class_PID PID_Omega;
    // PID Torque Loop
    Class_PID PID_Torque;

    void Init(CAN_HandleTypeDef *__hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method = Control_Method_ANGLE, int32_t __Encoder_Offset = 0, float __Omega_Max = 320.0f * RPM_TO_RADPS);

    uint16_t Get_Output_Max();
    Enum_CAN_Motor_Status Get_CAN_Motor_Status();
    float Get_Now_Angle();
    float Get_Now_Omega();
    float Get_Now_Torque();
    uint8_t Get_Now_Temperature();
    Enum_Control_Method Get_Control_Method();
    float Get_Target_Angle();
    float Get_Target_Omega();
    float Get_Target_Torque();
    float Get_Out();

    void Set_Control_Method(Enum_Control_Method __Control_Method);
    void Set_Target_Angle(float __Target_Angle);
    void Set_Target_Omega(float __Target_Omega);
    void Set_Target_Torque(float __Target_Torque);
    void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();

protected:
    //Variables related to initialization

    //Which CAN this motor uses (CAN1 or CAN2)
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //The Receiving CAN ID, C610/C620 motors 0x201~0x208, GM2060 0x205~0x20b
    Enum_CAN_Motor_ID CAN_ID;
    //The temporary place where the message to be sent is stored
    uint8_t *CAN_Tx_Data;
    //Encoder Offset - Used to set your own zero location.
    uint32_t Encoder_Offset;
    //Maximum Output Angular Velocity allowed from the motor, used for open loop.
    float Omega_Max;

    //Constants

    //Encoder Steps Per Rotation
    uint16_t Encoder_Num_Per_Round = 8192;
    //Maximum output voltage
    uint16_t Output_Max = 30000; //!!!!! Need to check this

    //Internal Variables

    //The current message number, increments every time a new message is received
    uint32_t Flag = 0;
    //The previous message number
    uint32_t Pre_Flag = 0;

    //Received Encoder Location, 0~8191 (These "Received" values are  essentially what are being measured by the motors)
    uint16_t Rx_Encoder = 0;
    //Received Rotation Speed, rpm
    int16_t Rx_Omega = 0;
    //Received Torque, Target Torque, -30000~30000 (This is both the Received and the Target because, it is the torque that the motor is trying to achieve - what you set the target to)
    int16_t Rx_Torque = 0;
    //Received Temperature, C
    uint16_t Rx_Temperature = 0;

    //Previous Encoder Location
    uint16_t Pre_Encoder = 0;
    //Keeps Track of the total encoder number - used as a part of formula to achieve a countinuous transition from 0 to 4096 degrees
    int32_t Total_Encoder = 0;
    //Total number of full rotations it has completed
    int32_t Total_Round = 0;

    //Read Variable

    //Motor Status
    Enum_CAN_Motor_Status CAN_Motor_Status = CAN_Motor_Status_DISABLE;

    //Current Angle, rad
    float Now_Angle = 0.0f;
    //Current Angular Velocity, rad/s
    float Now_Omega = 0.0f;
    //Current Torque
    float Now_Torque = 0.0f;
    //Current Temperature, C
    uint8_t Now_Temperature = 0;

    //Write Variables

    //Read Write Variables

    //Motor Control Method
    Enum_Control_Method Control_Method = Control_Method_ANGLE;
    //Target Angle, rad
    float Target_Angle = 0.0f;
    //Target Angular Velocity, rad/s
    float Target_Omega = 0.0f;
    //Target Torque
    float Target_Torque = 0.0f;
    //Output
    float Out = 0.0f;

    //Internal Function

    void Output();
};

/**
 * @brief C610Brush-Less Motor Driver, has a built in torque loop, MCU outputs torque
 *
 */
class Class_Motor_C610
{
public:
    // PID Angle Loop
    Class_PID PID_Angle;
    // PID Angular Velocity Loop
    Class_PID PID_Omega;

    void Init(CAN_HandleTypeDef *__hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method = Control_Method_OMEGA, float __Gearbox_Rate = 36.0f, float __Torque_Max = 10000.0f);

    uint16_t Get_Output_Max();
    Enum_CAN_Motor_Status Get_CAN_Motor_Status();
    float Get_Now_Angle();
    float Get_Now_Omega();
    float Get_Now_Torque();
    uint8_t Get_Now_Temperature();
    Enum_Control_Method Get_Control_Method();
    float Get_Target_Angle();
    float Get_Target_Omega();
    float Get_Target_Torque();
    float Get_Out();

    void Set_Control_Method(Enum_Control_Method __Control_Method);
    void Set_Target_Angle(float __Target_Angle);
    void Set_Target_Omega(float __Target_Omega);
    void Set_Target_Torque(float __Target_Torque);
    void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();

protected:
    //Variables related to initialization

    //Which CAN this motor uses (CAN1 or CAN2)
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //The Receiving CAN ID, C610/C620 motors 0x201~0x208, GM2060 0x205~0x20b
    Enum_CAN_Motor_ID CAN_ID;
    //The temporary place where the message to be sent is stored
    uint8_t *CAN_Tx_Data;
    //Gear Reduction Ratio, Default comes with Gear box
    float Gearbox_Rate = 36.0f;
    //Maximum Output Torque allowed from the motor, used for open loop.
    float Torque_Max = 10000.0f;

    //Constants

    //Encoder Steps Per Rotation
    uint16_t Encoder_Num_Per_Round = 8192;
    //Max Output Torque
    uint16_t Output_Max = 10000;

    //Internal Variables

    //Current message number
    uint32_t Flag = 0;
    //Previous message number
    uint32_t Pre_Flag = 0;

    //Received Encoder Position
    uint16_t Rx_Encoder = 0;
    //Received Rotation Velocity, rpm
    int16_t Rx_Omega = 0;
    //Received Torque, Target Torque -30000~30000
    int16_t Rx_Torque = 0;
    //Received Temperature, C
    uint16_t Rx_Temperature = 0;

    //Previous Encoder Position
    uint16_t Pre_Encoder = 0;
    //Total Encoder Count
    int32_t Total_Encoder = 0;
    //Total Rounds of Rotation Completed
    int32_t Total_Round = 0;

    //Read Variables

    //Motor Status
    Enum_CAN_Motor_Status CAN_Motor_Status = CAN_Motor_Status_DISABLE;

    //Current Angle, rad
    float Now_Angle = 0.0f;
    //Current Angular Velocity, rad/s
    float Now_Omega = 0.0f;
    //Current Torque, Measured by the Motor
    float Now_Torque = 0.0f;
    //Current Temperature, C
    uint8_t Now_Temperature = 0;

    //Write Variable

    //Read and Write Variable

    //Motor Control Method
    Enum_Control_Method Control_Method = Control_Method_ANGLE;
    //Target Angle, rad
    float Target_Angle = 0.0f;
    //Target Angular Velocity, rad/s
    float Target_Omega = 0.0f;
    //Target Torque, measured by the motor
    float Target_Torque = 0.0f;
    //Output
    float Out = 0.0f;

    //Internal Function

    void Output();
};

/**
 * @brief C620无刷电调, 自带扭矩环, 单片机控制输出扭矩
 *
 */
class Class_Motor_C620
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(CAN_HandleTypeDef *__hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method = Control_Method_OMEGA, float __Gearbox_Rate = 3591.0f / 187.0f, float __Torque_Max = 16384.0f);

    uint16_t Get_Output_Max();
    Enum_CAN_Motor_Status Get_CAN_Motor_Status();
    float Get_Now_Angle();
    float Get_Now_Omega();
    float Get_Now_Torque();
    uint8_t Get_Now_Temperature();
    Enum_Control_Method Get_Control_Method();
    float Get_Target_Angle();
    float Get_Target_Omega();
    float Get_Target_Torque();
    float Get_Out();

    void Set_Control_Method(Enum_Control_Method __Control_Method);
    void Set_Target_Angle(float __Target_Angle);
    void Set_Target_Omega(float __Target_Omega);
    void Set_Target_Torque(float __Target_Torque);
    void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_CAN_Motor_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //减速比, 默认带减速箱
    float Gearbox_Rate = 3591.0f / 187.0f;
    //最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
    float Torque_Max = 16384.0f;

    //常量

    //一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;
    //最大输出扭矩
    uint16_t Output_Max = 16384;

    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    //接收的编码器位置, 0~8191
    uint16_t Rx_Encoder = 0;
    //接收的速度, rpm
    int16_t Rx_Omega = 0;
    //接收的扭矩, 目标的扭矩, -30000~30000
    int16_t Rx_Torque = 0;
    //接收的温度, 摄氏度
    uint16_t Rx_Temperature = 0;

    //之前的编码器位置
    uint16_t Pre_Encoder = 0;
    //总编码器位置
    int32_t Total_Encoder = 0;
    //总圈数
    int32_t Total_Round = 0;

    //读变量

    //电机状态
    Enum_CAN_Motor_Status CAN_Motor_Status = CAN_Motor_Status_DISABLE;

    //当前的角度, rad
    float Now_Angle = 0.0f;
    //当前的速度, rad/s
    float Now_Omega = 0.0f;
    //当前的扭矩, 直接采用反馈值
    float Now_Torque = 0.0f;
    //当前的温度, 摄氏度
    uint8_t Now_Temperature = 0;

    //写变量

    //读写变量

    //电机控制方式
    Enum_Control_Method Control_Method = Control_Method_ANGLE;
    //目标的角度, rad
    float Target_Angle = 0.0f;
    //目标的速度, rad/s
    float Target_Omega = 0.0f;
    //目标的扭矩, 直接采用反馈值
    float Target_Torque = 0.0f;
    //输出量
    float Out = 0.0f;

    //内部函数

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
