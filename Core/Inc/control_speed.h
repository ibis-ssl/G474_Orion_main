#ifndef CONTROL_SPEED_H_
#define CONTROL_SPEED_H_

#include "management.h"
#include "robot_packet.h"

void setTargetAccel(RobotCommandV2 * ai_cmd, accel_vector_t * acc_vel);
void accelControl(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni);
void speedControl(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni, RobotCommandV2 * ai_cmd);

#endif