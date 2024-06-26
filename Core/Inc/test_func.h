/*
 * test_func.h
 *
 *  Created on: Apr 19, 2024
 *      Author: hiroyuki
 */

#ifndef INC_TEST_FUNC_H_
#define INC_TEST_FUNC_H_

#include "management.h"

bool isLatencyCheckModeEnabled(system_t * sys, debug_t * debug, ai_cmd_t * ai_cmd);
float getTargetThetaInLatencyCheckMode(debug_t * debug, ai_cmd_t * ai_cmd);
void motor_test(system_t * sys, output_t * output);
void dribbler_test(system_t * sys, output_t * output);
void kicker_test(system_t * sys, can_raw_t * can_raw, bool manual_mode, output_t * output);
void motor_calibration(system_t * sys);

#endif /* INC_TEST_FUNC_H_ */
