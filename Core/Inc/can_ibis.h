/*
 * can_ibis.h
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */

#ifndef CAN_IBIS_H_
#define CAN_IBIS_H_

#include "management.h"
#include "robot_packet.h"

typedef struct
{
  uint32_t motor_call_samples;
  uint32_t motor_call_delta_sum_us;
  uint16_t motor_call_delta_max_us;
  uint16_t motor_call_delta_last_us;
  uint32_t tx_add_ok[2];
  uint32_t tx_add_error[2];
  uint32_t sw_buffered[2];
  uint32_t sw_dropped[2];
} can_tx_debug_t;

extern volatile can_tx_debug_t can_tx_debug;

void can1_init_ibis(FDCAN_HandleTypeDef * handler);
void can1_send(int id, uint8_t senddata[]);
void can2_init_ibis(FDCAN_HandleTypeDef * handler);
void can2_send(int id, uint8_t senddata[]);

void canTxEmptyInterrupt(FDCAN_HandleTypeDef * hfdcan);

void parseCanCmd(uint16_t rx_can_id, uint8_t rx_data[], can_raw_t * can_raw, system_t * sys, motor_t * motor, mouse_t * mouse);
void sendActuatorCanCmdRun(RobotCommandV2 * ai_cmd, system_t * sys, can_raw_t * can_raw);
void sendActuatorCanCmdStop();
void sendCanError();
void resetPowerBoard();

bool canRxTimeoutDetection(can_raw_t * can_raw);
void canRxTimeoutCntCycle(can_raw_t * can_raw);
bool allEncInitialized(can_raw_t * can_raw);

#endif /* CAN_IBIS_H_ */
