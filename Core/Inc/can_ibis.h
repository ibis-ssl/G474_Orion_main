/*
 * can_ibis.h
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */

#ifndef CAN_IBIS_H_
#define CAN_IBIS_H_

#include "management.h"



void can1_init_ibis(FDCAN_HandleTypeDef* handler);
void can1_send(int id, uint8_t senddata[]);
void can2_init_ibis(FDCAN_HandleTypeDef* handler);
void can2_send(int id, uint8_t senddata[]);

uint32_t canTimeoutCnt();

#endif /* CAN_IBIS_H_ */
