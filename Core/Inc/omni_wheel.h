/*
 * omni_wheel.h
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */

#ifndef OMNI_WHEEL_H_
#define OMNI_WHEEL_H_


#include "management.h"
#include <stdbool.h>
#include <string.h>

void omni_move(float32_t vel_x_omni,float32_t vel_y_omni,float32_t omega_omni,float32_t duty_Limit);
void stall_move();
#endif /* OMNI_WHEEL_H_ */
