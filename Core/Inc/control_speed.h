#ifndef CONTROL_SPEED_H_
#define CONTROL_SPEED_H_

#include "management.h"
#include "robot_packet.h"

void setTargetAccel(RobotCommandV2 * ai_cmd, accel_vector_t * acc_vel, bool local_devvel_control_flag);
void accelControl(accel_vector_t * acc_vel, target_t * target, bool local_devvel_control_flag);
void speedControl(accel_vector_t * acc_vel, target_t * target, imu_t * imu);

#endif