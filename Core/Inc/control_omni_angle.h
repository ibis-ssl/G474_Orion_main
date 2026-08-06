#ifndef CONTROL_OMNI_ANGLE_H_
#define CONTROL_OMNI_ANGLE_H_

#include "management.h"
#include "robot_packet.h"

void setTargetOmniAngle(target_t * target);
void clearOmniRotationAngleErrorIfStopped(const RobotCommandV2 * ai_cmd, const imu_t * imu, const system_t * sys, target_t * target, const motor_t * motor);
void omniAngleControl(target_t * target, output_t * output, motor_t * motor);

#endif
