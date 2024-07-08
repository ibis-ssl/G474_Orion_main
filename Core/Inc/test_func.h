/*
 * test_func.h
 *
 *  Created on: Apr 19, 2024
 *      Author: hiroyuki
 */

#ifndef INC_TEST_FUNC_H_
#define INC_TEST_FUNC_H_

#include "management.h"
#include "robot_packet.h"

bool isLatencyCheckModeEnabled(system_t * sys, debug_t * debug, RobotCommandV2 * ai_cmd);
float getTargetThetaInLatencyCheckMode(debug_t * debug, RobotCommandV2 * ai_cmd);
void motorTest(system_t * sys, output_t * output);
void dribblerTest(system_t * sys, output_t * output);
void kickerTest(system_t * sys, can_raw_t * can_raw, bool manual_mode, output_t * output);
void motorCalibration(system_t * sys);

#endif /* INC_TEST_FUNC_H_ */
