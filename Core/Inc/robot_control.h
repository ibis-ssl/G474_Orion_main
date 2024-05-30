/*
 * robot_control.h
 *
 *  Created on: May 19, 2024
 *      Author: hiroyuki
 */

#ifndef INC_ROBOT_CONTROL_H_
#define INC_ROBOT_CONTROL_H_

#include "management.h"

void theta_control(float target_theta, accel_vector_t * acc_vel, output_t * output, imu_t * imu);

void local_feedback(integration_control_t * integ, imu_t * imu, system_t * sys, target_t * target, ai_cmd_t * ai_cmd, omni_t * omni);
void accel_control(accel_vector_t * acc_vel, output_t * output, target_t * target,omni_t *omni);
void speed_control(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni);
void output_limit(output_t * output, debug_t * debug);

#endif /* INC_ROBOT_CONTROL_H_ */
