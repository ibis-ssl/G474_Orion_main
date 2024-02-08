/*
 * can_ibis.c
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */

#include "can_ibis.h"

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_FilterTypeDef sFilterConfig;

// power,FC,mose
void can1_init_ibis(FDCAN_HandleTypeDef * handler)
{
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
}

void can1_send(int id, uint8_t senddata[])
{
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  /* Request transmission */
  //if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 3) return;
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 3) {
  }
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, senddata);
}

void can2_init_ibis(FDCAN_HandleTypeDef * handler)
{
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
}

void can2_send(int id, uint8_t senddata[])
{
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  /* Request transmission */
  //if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 3) return;
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) != 3) {
  }
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, senddata);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs)
{
  uint8_t RxData[CAN_RX_DATA_SIZE];
  FDCAN_RxHeaderTypeDef RxHeader;
  uint16_t rx_can_id;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
      Error_Handler();
    }
    rx_can_id = RxHeader.Identifier;
    switch (rx_can_id) {
      // error
      case 0x000:
      case 0x001:
        can_raw.error_no[0] = RxData[0];
        can_raw.error_no[1] = RxData[1];
        can_raw.error_no[2] = RxData[2];
        can_raw.error_no[3] = RxData[3];
        sys.error_id = (uint16_t)((RxData[1] << 8) | RxData[0]);
        sys.error_info = (uint16_t)((RxData[3] << 8) | RxData[2]);
        sys.error_value = uchar4_to_float(&RxData[4]);
        sys.error_flag = true;
        break;

      // can_raw.motor_feedback
      case 0x200:
      case 0x201:
      case 0x202:
      case 0x203:
        motor.enc_angle[rx_can_id - 0x200] = uchar4_to_float(&RxData[4]);
        can_raw.motor_feedback[rx_can_id - 0x200] = uchar4_to_float(RxData);
        can_raw.motor_feedback_velocity[rx_can_id - 0x200] = can_raw.motor_feedback[3] * OMNI_DIAMETER * M_PI;
        break;

        // can_raw.power_Voltage
      case 0x210:  // m0
      case 0x211:  // m1
      case 0x212:  // m2
      case 0x213:  // m3
      case 0x214:  // sub board (not used)
      case 0x215:  // battery
      case 0x216:  // capacitor
        can_raw.power_voltage[rx_can_id - 0x210] = uchar4_to_float(RxData);
        break;

      // can_raw.temperature from BLDC
      case 0x220:
      case 0x221:
      case 0x222:
      case 0x223:
        can_raw.temperature[rx_can_id - 0x220] = uchar4_to_float(RxData);
        break;
      // can_raw.temperature from power
      case 0x224:
        can_raw.temperature[4] = RxData[0];  // fet
        can_raw.temperature[5] = RxData[1];  // coil 1
        can_raw.temperature[6] = RxData[2];  // coil 2
        break;

      // can_raw.current
      case 0x230:
      case 0x231:
      case 0x232:
      case 0x233:
      case 0x234:
        can_raw.current[rx_can_id - 0x230] = uchar4_to_float(RxData);
        break;

      // can_raw.ball_detection
      case 0x240:
        can_raw.ball_detection[0] = RxData[0];
        can_raw.ball_detection[1] = RxData[1];
        can_raw.ball_detection[2] = RxData[2];
        can_raw.ball_detection[3] = RxData[3];
        break;

      // mouseXY
      case 0x241:
        mouse.raw[0] = (int16_t)((RxData[1] << 8) | RxData[0]);
        mouse.raw[1] = (int16_t)((RxData[3] << 8) | RxData[2]);
        mouse.quality = (uint16_t)((RxData[5] << 8) | RxData[4]);
        mouseOdometory();
        mouse.loop_cnt_debug = mouse.integral_loop_cnt;
        mouse.integral_loop_cnt = 0;

        // 持ち上げ･コート外検知
        /*if (mouse.quality < 30 && sys.system_time_ms > 1000) {
          error_flag = true;
        }*/
        break;
    }
  }
}
