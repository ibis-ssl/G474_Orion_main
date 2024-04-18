/*
 * ai_comm.h
 *
 *  Created on: Apr 19, 2024
 *      Author: hiroyuki
 */

#ifndef INC_AI_COMM_H_
#define INC_AI_COMM_H_

#include "management.h"

void resetAiCmdData(ai_cmd_t * ai_cmd);
void parseRxCmd(connection_t * con, system_t * sys, ai_cmd_t * ai_cmd, integration_control_t * integ, uint8_t data[]);
void sendRobotInfo();

#endif /* INC_AI_COMM_H_ */
