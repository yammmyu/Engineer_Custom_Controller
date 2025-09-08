#include "motor_control.h"
#include "pid_controller.h"

// PID instances for the joint
static PID_t pid_pos;
static PID_t pid_vel;

void arm_control_init(void)
{
    // Outer: Position PID (deg → RPM)
    pid_init(&pid_pos,
             10.0f, 0.1f, 0.25f,    // Kp, Ki, Kd
             -200.0f, 200.0f,       // output limits = RPM
             -50.0f, 50.0f,         // integrator clamp
             0.5f);                 // deadband in deg

    // Middle: Velocity PID (RPM → normalized voltage [-1,1])
    pid_init(&pid_vel,
             0.035f, 0.3f, 0.002f,  // Kp, Ki, Kd
             -1.0f, 1.0f,           // output = normalized voltage demand
             -0.5f, 0.5f,           // integrator clamp
             0.0f);                 // deadband
}

int16_t arm_control_update(float pos_target_deg, float pos_meas_deg,
                           float vel_meas_rpm, float dt)
{
    // Outer loop: Position → desired velocity
    float vel_target = pid_update(&pid_pos, pos_target_deg, pos_meas_deg, dt);

    // Middle loop: Velocity → normalized voltage
    float volt_demand = pid_update(&pid_vel, vel_target, vel_meas_rpm, dt);

    // Convert normalized voltage [-1,1] → CAN cmd [-25000, 25000]
    return voltage_to_can(volt_demand);
}
