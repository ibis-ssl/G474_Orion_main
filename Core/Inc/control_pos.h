#ifndef CONTROL_POS_H_
#define CONTROL_POS_H_

#include "management.h"
#include "robot_packet.h"

void localPositionFeedback(integration_control_t * integ, imu_t * imu, target_t * target, RobotCommandV2 * ai_cmd, omni_t * omni, mouse_t * mouse, accel_vector_t * acc_vel, output_t * output);

#endif