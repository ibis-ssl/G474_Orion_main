/*
 * state_func.h
 *
 *  Created on: Apr 19, 2024
 *      Author: hiroyuki
 */

#ifndef INC_STATE_FUNC_H_
#define INC_STATE_FUNC_H_

#include "management.h"
#include "robot_packet.h"

void motorTest(system_t * sys, output_t * output, omni_t * omni);
void dribblerTest(system_t * sys, output_t * output, can_raw_t * can_raw);
void kickerTest(system_t * sys, can_raw_t * can_raw, bool manual_mode, output_t * output);
void latencyCheck(system_t * sys, debug_t * debug, RobotCommandV2 * ai_cmd, output_t * output, imu_t * imu);
void motorCalibration(system_t * sys);
void maintaskRun(
  system_t * sys, RobotCommandV2 * ai_cmd, imu_t * imu, accel_vector_t * acc_vel, integration_control_t * integ, target_t * target, omni_t * omni, mouse_t * mouse, debug_t * debug, output_t * output,
  can_raw_t * can_raw);
void maintaskStop(output_t * output);

#endif /* INC_STATE_FUNC_H_ */
