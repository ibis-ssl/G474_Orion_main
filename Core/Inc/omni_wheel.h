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

void omni_move(float vel_x_omni,float vel_y_omni,float omega_omni,float duty_Limit);
#endif /* OMNI_WHEEL_H_ */
