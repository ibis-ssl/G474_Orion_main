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

void omniMove(output_t * output, float out_limit);
void omniStopAll(output_t * output);

#endif /* OMNI_WHEEL_H_ */
