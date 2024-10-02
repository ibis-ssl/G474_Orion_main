#ifndef _BUZZER_CONTROL_H_
#define _BUZZER_CONTROL_H_

#include "management.h"
#include "robot_packet.h"
float getBatteryRemain(can_raw_t * can_raw);
bool isLowVoltage(can_raw_t * can_raw);
bool isVisionLost(system_t * sys, connection_t * con, RobotCommandV2 * ai_cmd);
void buzzerControl(can_raw_t * can_raw, system_t * sys, connection_t * con, RobotCommandV2 * ai_cmd);

#endif