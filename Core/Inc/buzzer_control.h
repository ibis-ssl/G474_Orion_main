#ifndef _BUZZER_CONTROL_H_
#define _BUZZER_CONTROL_H_

#include "management.h"
#include "robot_packet.h"

void buzzerControl(can_raw_t * can_raw, system_t * sys, connection_t * con, RobotCommandV2 * ai_cmd);

#endif