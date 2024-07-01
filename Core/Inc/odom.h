/*
 * odom.h
 *
 *  Created on: Nov 17, 2023
 *      Author: hiroyuki
 */

#ifndef INC_ODOM_H_
#define INC_ODOM_H_
#include "management.h"

void omniOdometry(ai_cmd_t * ai_cmd, motor_t * motor, omni_t * omni, integration_control_t * integ, connection_t * connection, imu_t * imu);
void mouseOdometry(mouse_t * mouse, imu_t * imu);

void convertGlobalToLocal(float global[], float local[], float yaw_rad);
void convertLocalToGlobal(float local[], float global[], float yaw_rad);

#endif /* INC_ODOM_H_ */
