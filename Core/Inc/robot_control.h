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

void setLocalTargetSpeed(RobotCommandV2 * ai_cmd, target_t * target, imu_t * imu);
void setOutZero(output_t * output);
void clearSpeedContrlValue(accel_vector_t * acc_vel, target_t * target, imu_t * imu, omni_t * omni, motor_t * motor);

#endif /* INC_ROBOT_CONTROL_H_ */
