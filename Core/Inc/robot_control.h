/*
 * robot_control.h
 *
 *  Created on: May 19, 2024
 *      Author: hiroyuki
 */

#ifndef INC_ROBOT_CONTROL_H_
#define INC_ROBOT_CONTROL_H_

#include "management.h"
#include "robot_packet.h"

void robotControl(
  system_t * sys, RobotCommandV2 * ai_cmd, imu_t * imu, accel_vector_t * acc_vel, integration_control_t * integ, target_t * target, omni_t * omni, mouse_t * mouse, debug_t * debug, output_t * output,
  omega_target_t * omega_target);

#endif /* INC_ROBOT_CONTROL_H_ */
