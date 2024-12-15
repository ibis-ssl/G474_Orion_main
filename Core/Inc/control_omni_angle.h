#ifndef CONTROL_OMNI_ANGLE_H_
#define CONTROL_OMNI_ANGLE_H_

#include "management.h"
#include "robot_packet.h"

void setTargetOmniAngle(target_t * target);
void omniAngleControl(target_t * target, output_t * output, motor_t * motor);

#endif