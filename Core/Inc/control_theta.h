#ifndef CONTROL_THETA_H_
#define CONTROL_THETA_H_

#include "management.h"
#include "robot_packet.h"

void thetaControl(RobotCommandV2 * ai_cmd, output_t * output, imu_t * imu, omega_target_t * omega_target);

#endif