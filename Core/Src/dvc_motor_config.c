#include "dvc_motor_config.h"
#include "cfg_pid_params.h"
#include "stm32f4xx_hal.h"
#include "main.h"


Motor_t motors[MOTOR_COUNT];

void Motors_Init(void)
{
    // Example CAN IDs and methods per motor
    Enum_CAN_Motor_ID motor_ids[MOTOR_COUNT] = {
        CAN_Motor_ID_0x201, // motor0
        CAN_Motor_ID_0x202, // motor1
        CAN_Motor_ID_0x203, // motor2
        CAN_Motor_ID_0x205, // motor3
        CAN_Motor_ID_0x206, // motor4
        CAN_Motor_ID_0x207  // motor5
    };

    Enum_Control_Method methods[MOTOR_COUNT] = {
        Control_Method_TORQUE, // motor0
        Control_Method_TORQUE, // motor1
        Control_Method_TORQUE, // motor2
        Control_Method_ANGLE,  // motor3
        Control_Method_ANGLE,  // motor4
        Control_Method_ANGLE   // motor5
    };

	float encoder_offsets[MOTOR_COUNT] = {
		0.0f, // motor0
		0.0f, // motor1
		0.0f, // motor2
		0.0f, // motor3
		0.0f, // motor4
		0.0f  // motor5
	};

    for (int i = 0; i < MOTOR_COUNT; i++) {
        // Init motor struct
        Motor_Init(&motors[i],
                   &hcan2,               // CAN bus (could mix hcan1/hcan2 if needed)
                   motor_ids[i],
                   methods[i],
                   1.0f,                 // gearbox ratio (adjust per motor)
                   1.0f,                 // torque_max
                   400.0f,               // omega_max
                   encoder_offsets[i]);                   // encoder offset

        // Angle PID
        pid_init(&motors[i].PID_Angle,
                 motor_pid_cfg[i].angle.Kp,
                 motor_pid_cfg[i].angle.Ki,
                 motor_pid_cfg[i].angle.Kf,
				 motor_pid_cfg[i].angle.Kd,
                 motor_pid_cfg[i].angle.i_out_max,
                 motor_pid_cfg[i].angle.out_max,
                 motor_pid_cfg[i].angle.dt,
                 motor_pid_cfg[i].angle.dead_zone,
                 motor_pid_cfg[i].angle.i_var_a,
                 motor_pid_cfg[i].angle.i_var_b,
                 motor_pid_cfg[i].angle.i_sep_threshold,
                 motor_pid_cfg[i].angle.d_first);

        // Torque PID
        pid_init(&motors[i].PID_Torque,
                 motor_pid_cfg[i].torque.Kp,
                 motor_pid_cfg[i].torque.Ki,
                 motor_pid_cfg[i].torque.Kf,
				 motor_pid_cfg[i].angle.Kd,
                 motor_pid_cfg[i].torque.i_out_max,
                 motor_pid_cfg[i].torque.out_max,
                 motor_pid_cfg[i].torque.dt,
                 motor_pid_cfg[i].torque.dead_zone,
                 motor_pid_cfg[i].torque.i_var_a,
                 motor_pid_cfg[i].torque.i_var_b,
                 motor_pid_cfg[i].torque.i_sep_threshold,
                 motor_pid_cfg[i].torque.d_first);
    }
}
