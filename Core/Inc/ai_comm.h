/*
 * ai_comm.h
 *
 *  Created on: Apr 19, 2024
 *      Author: hiroyuki
 */

#ifndef INC_AI_COMM_H_
#define INC_AI_COMM_H_

#include "management.h"
#include "robot_packet.h"

void resetAiCmdData(RobotCommandV2 * ai_cmd);
void sendRobotInfo(
  can_raw_t * can_raw, system_t * sys, imu_t * imu, omni_t * omni, mouse_t * mouse, RobotCommandV2 * ai_cmd, connection_t * con, integration_control_t * integ, output_t * out, target_t * target);
void commStateCheck(connection_t * connection, system_t * sys, RobotCommandV2 * ai_cmd);
void resetLocalSpeedControl(RobotCommandV2 * ai_cmd);
void updateCM4CmdTimeStamp(connection_t * connection, system_t * sys);
void calcAIcmdRxFrq(connection_t * connection);
#endif /* INC_AI_COMM_H_ */
