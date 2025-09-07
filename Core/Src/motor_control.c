#include "pid.h"

// Declare PID instances
pid_t pid_pos = { .Kp=10, .Ki=0, .Kd=0.2, .out_min=-200, .out_max=200,
                  .integ_min=-20, .integ_max=20, .I_deadband=1.0 };
pid_t pid_vel = { .Kp=0.03, .Ki=0.3, .Kd=0.001, .out_min=-10, .out_max=10,
                  .integ_min=-5, .integ_max=5, .I_deadband=0 };
pid_t pid_cur = { .Kp=0.7, .Ki=300, .Kd=0, .out_min=-10, .out_max=10,
                  .integ_min=-5, .integ_max=5, .I_deadband=0 };

void control_loop(float dt_pos, float dt_vel, float dt_cur) {
    float target_angle = 30.0f;  // deg
    float current_angle = read_joint_angle();
    float target_velocity = pid_update(&pid_pos, target_angle, current_angle, dt_pos);

    float current_velocity = read_joint_velocity();
    float target_current = pid_update(&pid_vel, target_velocity, current_velocity, dt_vel);

    float motor_command;
    if (have_current_feedback()) {
        float current_measure = read_motor_current();
        motor_command = pid_update(&pid_cur, target_current, current_measure, dt_cur);
    } else {
        motor_command = target_current; // open-loop current
    }

    send_current_to_gm6020(motor_command);
}
