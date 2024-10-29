#ifndef CONTROL_THETA_H_
#define CONTROL_THETA_H_

#include "management.h"
#include "robot_packet.h"
void thetaControl(RobotCommandV2 * ai_cmd, imu_t * imu, target_t * target);
void yawFilter(system_t * sys, debug_t * debug, imu_t * imu, RobotCommandV2 * ai_cmd, connection_t * conn);

#endif