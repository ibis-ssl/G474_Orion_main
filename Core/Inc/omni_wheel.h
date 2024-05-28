/*
 * omni_wheel.h
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */

#ifndef OMNI_WHEEL_H_
#define OMNI_WHEEL_H_

#include <stdbool.h>
#include <string.h>

#include "management.h"

void omni_move(float vel_y_robot, float vel_x_robot, float omega_robot, float duty_limit, output_t * output);

#endif /* OMNI_WHEEL_H_ */
