#include <pid_alg.h>
#include "arm_control.h"

// PID instances for the joint
static PID_t pid_pos;
static PID_t pid_vel;

// Define low pass Filter
static float pos_filtered = 0.0f;
static float vel_filtered = 0.0f;

float lowpass_filter_pos(float input, float alpha)
{
    pos_filtered = alpha * input + (1.0f - alpha) * pos_filtered;
    return pos_filtered;
}

float lowpass_filter_vel(float input, float alpha)
{
    vel_filtered = alpha * input + (1.0f - alpha) * vel_filtered;
    return vel_filtered;
}


void arm_control_init(void)
{
    pid_init(&pid_pos,
             0.58f, 0.0f, 0.001f,    // Kp, Ki, Kd
             -999.0f, 999.0f,       // output limits = RPM
             -10.0f, 10.0f,         // integrator clamp
             0.3f);                 // deadband in deg

    // Middle: Velocity PID (RPM → normalized voltage [-1,1])
    pid_init(&pid_vel,
             0.036f, 0.0f, 0.0f,  // Kp, Ki, Kd
             -0.3f, 0.3f,           // output = normalized voltage demand
             -0.4f, 0.4f,           // integrator clamp
             0.0f);                 // deadband

	/*
	// Outer: Position PID (deg → RPM)
    pid_init(&pid_pos,
             2.0f, 0.00f, 0.01f,    // Kp, Ki, Kd
             -180.0f, 180.0f,       // output limits = RPM
             -10.0f, 10.0f,         // integrator clamp
             0.3f);                 // deadband in deg

    // Middle: Velocity PID (RPM → normalized voltage [-1,1])
    pid_init(&pid_vel,
             0.008f, 0.0f, 0.0f,  // Kp, Ki, Kd
             -0.2f, 0.2f,           // output = normalized voltage demand
             -0.05f, 0.05f,           // integrator clamp
             0.0f);                 // deadband
    */
}

int16_t arm_control_update(float pos_target_deg, float pos_meas_deg,
                           float vel_meas_rpm, float dt)
{
    // Angle: weaker filter (more responsive)
    float f_pos_meas_deg = lowpass_filter_pos(pos_meas_deg, 0.90f); // α=0.2

    // Outer loop: Position → desired velocity
    float vel_target = pid_update(&pid_pos, pos_target_deg, f_pos_meas_deg, dt);

    // Speed: stronger filter (smoother)
    float f_vel_meas_rpm = lowpass_filter_vel(vel_meas_rpm, 0.1f); // α=0.1

    // Middle loop: Velocity → normalized voltage
    float volt_demand = pid_update(&pid_vel, vel_target, f_vel_meas_rpm, dt);

    // Convert normalized voltage [-1,1] → CAN cmd [-25000, 25000]
    return voltage_to_can(volt_demand);
}
