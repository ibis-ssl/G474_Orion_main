/*
 * myatan2.h
 *
 *  Created on: 2018/01/10
 *      Author: okada_tech
 */

#ifndef MYATAN2_H_
#define MYATAN2_H_

#include <stdbool.h>
#include <string.h>

#define TABLE_SIXE_ATAN   (577)
#define BLOCK_NUM_ATAN    (26)
#define DEBIDE_ONE_BLOCK_ATAN  (24)

float myAtan(float x);
float myAtan2(float y, float x);



#endif /* MYATAN2_H_ */
