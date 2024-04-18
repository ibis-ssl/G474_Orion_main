/*
 * test_func.h
 *
 *  Created on: Apr 19, 2024
 *      Author: hiroyuki
 */

#ifndef INC_TEST_FUNC_H_
#define INC_TEST_FUNC_H_

#include "management.h"

void motor_test(system_t * sys);
void dribbler_test(system_t * sys);
void kicker_test(system_t * sys, can_raw_t * can_raw, bool manual_mode);
void motor_calibration(system_t * sys);

#endif /* INC_TEST_FUNC_H_ */
