#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

void arm_control_init(void);
void arm_control_update(float pos_target_deg, float pos_meas_deg,
                        float vel_meas_rpm, float dt);

#endif // ARM_CONTROL_H
