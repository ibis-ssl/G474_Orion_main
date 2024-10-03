/*
 * odom.h
 *
 *  Created on: Nov 17, 2023
 *      Author: hiroyuki
 */

#ifndef INC_ODOM_H_
#define INC_ODOM_H_
#include "management.h"
#include "robot_packet.h"

void omniOdometryUpdate(motor_t * motor, omni_t * omni, imu_t * imu);
void mouseOdometryUpdate(mouse_t * mouse, imu_t * imu);
void inntegOdomUpdate(RobotCommandV2 * ai_cmd, omni_t * omni, integ_control_t * integ, connection_t * connection, imu_t * imu, system_t * sys);

#endif /* INC_ODOM_H_ */
