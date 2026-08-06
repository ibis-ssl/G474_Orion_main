/*
 * can_ibis.c
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */

#include "can_ibis.h"

#include "actuator.h"
#include "omni_wheel.h"
#include "util.h"

FDCAN_FilterTypeDef sFilterConfig;

/******************* can送信バッファ ***************** */

typedef struct
{
  uint32_t id;
  uint8_t data[8];
} buffer_data_t;

typedef struct
{
  buffer_data_t * buffer;
  uint8_t size;
  volatile uint8_t head;
  volatile uint8_t tail;
} fifo_buffer_t;

#define CAN_BUF_CAPACITY (20U)
#define CAN_BUF_STORAGE_SIZE (CAN_BUF_CAPACITY + 1U)
buffer_data_t can1_buf_data[CAN_BUF_STORAGE_SIZE], can2_buf_data[CAN_BUF_STORAGE_SIZE];
fifo_buffer_t can_buf[2];

uint32_t can_resend_cnt = 0;
volatile can_tx_debug_t can_tx_debug = {0};
static uint16_t motor_right_call_time_us = 0;

static inline uint16_t elapsedTim7Us(uint16_t start, uint16_t end)
{
  const uint16_t period = (uint16_t)(htim7.Init.Period + 1U);
  return (end >= start) ? (end - start) : (uint16_t)(period - start + end);
}

static inline bool canBufferEmpty(const fifo_buffer_t * buf)
{
  return buf->head == buf->tail;
}

static inline uint8_t canBufferNextIndex(const fifo_buffer_t * buf, uint8_t index)
{
  index++;
  return index < buf->size ? index : 0U;
}

static inline bool canBufferEnqueue(fifo_buffer_t * buf, const buffer_data_t * data)
{
  const uint8_t next_head = canBufferNextIndex(buf, buf->head);
  if (next_head == buf->tail) {
    return false;
  }

  buf->buffer[buf->head] = *data;
  __DMB();
  buf->head = next_head;
  return true;
}

static inline const buffer_data_t * canBufferFront(const fifo_buffer_t * buf)
{
  return &buf->buffer[buf->tail];
}

static inline void canBufferPop(fifo_buffer_t * buf)
{
  __DMB();
  buf->tail = canBufferNextIndex(buf, buf->tail);
}

/******************* can送信バッファ ***************** */

// power,FC,mouse
void can1_init_ibis(FDCAN_HandleTypeDef * handler)
{
  can_buf[0].buffer = can1_buf_data;
  can_buf[0].size = CAN_BUF_STORAGE_SIZE;
  can_buf[0].head = 0;
  can_buf[0].tail = 0;

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
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_TX_BUFFER0) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_TX_BUFFER1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_TX_BUFFER2) != HAL_OK) {
    Error_Handler();
  }
}

void can2_init_ibis(FDCAN_HandleTypeDef * handler)
{
  can_buf[1].buffer = can2_buf_data;
  can_buf[1].size = CAN_BUF_STORAGE_SIZE;
  can_buf[1].head = 0;
  can_buf[1].tail = 0;

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
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_TX_BUFFER0) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_TX_BUFFER1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_TX_BUFFER2) != HAL_OK) {
    Error_Handler();
  }
}

inline void can1_send(int id, uint8_t senddata[])
{
  if (id == 0x100) {
    motor_right_call_time_us = (uint16_t)htim7.Instance->CNT;
  }
  FDCAN_TxHeaderTypeDef tx_header = {
    .Identifier = (uint32_t)id,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0,
  };

  buffer_data_t data;
  /* Request transmission */
  //if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 3) return;
  if (!canBufferEmpty(&can_buf[0]) || HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
    data.id = id;
    memcpy(data.data, senddata, 8);
    if (canBufferEnqueue(&can_buf[0], &data)) {
      can_tx_debug.sw_buffered[0]++;
    } else {
      can_tx_debug.sw_dropped[0]++;
    }
  } else {
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, senddata) == HAL_OK) {
      can_tx_debug.tx_add_ok[0]++;
    } else {
      can_tx_debug.tx_add_error[0]++;
    }
  }
}

inline void can2_send(int id, uint8_t senddata[])
{
  if (id == 0x102) {
    const uint16_t delta_us = elapsedTim7Us(motor_right_call_time_us, (uint16_t)htim7.Instance->CNT);
    can_tx_debug.motor_call_delta_last_us = delta_us;
    can_tx_debug.motor_call_delta_sum_us += delta_us;
    can_tx_debug.motor_call_samples++;
    if (delta_us > can_tx_debug.motor_call_delta_max_us) {
      can_tx_debug.motor_call_delta_max_us = delta_us;
    }
  }
  FDCAN_TxHeaderTypeDef tx_header = {
    .Identifier = (uint32_t)id,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0,
  };

  buffer_data_t data;
  /* Request transmission */
  //if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 3) return;
  if (!canBufferEmpty(&can_buf[1]) || HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0) {
    data.id = id;
    memcpy(data.data, senddata, 8);
    if (canBufferEnqueue(&can_buf[1], &data)) {
      can_tx_debug.sw_buffered[1]++;
    } else {
      can_tx_debug.sw_dropped[1]++;
    }
  } else {
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, senddata) == HAL_OK) {
      can_tx_debug.tx_add_ok[1]++;
    } else {
      can_tx_debug.tx_add_error[1]++;
    }
  }
}

inline void canTxEmptyInterrupt(FDCAN_HandleTypeDef * hfdcan)
{
  const uint8_t bus_index = hfdcan->Instance == FDCAN1 ? 0U : 1U;
  fifo_buffer_t * const fifo = &can_buf[bus_index];

  while (!canBufferEmpty(fifo) && HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0U) {
    const buffer_data_t * const data = canBufferFront(fifo);
    FDCAN_TxHeaderTypeDef tx_header = {
      .Identifier = data->id,
      .IdType = FDCAN_STANDARD_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = FDCAN_DLC_BYTES_8,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0,
    };

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t *)data->data) != HAL_OK) {
      can_tx_debug.tx_add_error[bus_index]++;
      break;
    }

    can_tx_debug.tx_add_ok[bus_index]++;
    canBufferPop(fifo);
  }
}

inline void parseCanCmd(uint16_t rx_can_id, uint8_t rx_data[], can_raw_t * can_raw, system_t * sys, motor_t * motor, mouse_t * mouse)
{
  switch (rx_can_id) {
    // error
    case 0x000:
    case 0x001:
      can_raw->error_no[0] = rx_data[0];
      can_raw->error_no[1] = rx_data[1];
      can_raw->error_no[2] = rx_data[2];
      can_raw->error_no[3] = rx_data[3];
      sys->current_error.id = (uint16_t)((rx_data[1] << 8) | rx_data[0]);
      sys->current_error.info = (uint16_t)((rx_data[3] << 8) | rx_data[2]);
      sys->current_error.value = uchar4_to_float(&rx_data[4]);
      sys->latest_error.id = sys->current_error.id;
      sys->latest_error.info = sys->current_error.info;
      sys->latest_error.value = sys->current_error.value;
      sys->error_flag = true;
      break;

    // can_raw->motor_feedback
    case 0x200:
    case 0x201:
    case 0x202:
    case 0x203:
    case 0x204:
      uint32_t enc_id = rx_can_id - 0x200;
      motor->rps[enc_id] = uchar4_to_float(rx_data);
      motor->angle_rad[enc_id] = -uchar4_to_float(&rx_data[4]);  //エンコーダ角度とモーター回転方向は逆なのでここで吸収

      can_raw->motor_feedback[enc_id] = motor->rps[enc_id];

      if (rx_can_id == 0x200 || rx_can_id == 0x201) {
        can_raw->rx_stat.timeout_cnt[BOARD_ID_MOTOR_RIGHT] = 0;
      } else if (rx_can_id == 0x202 || rx_can_id == 0x203) {
        can_raw->rx_stat.timeout_cnt[BOARD_ID_MOTOR_LEFT] = 0;
      }
      can_raw->rx_stat.enc_flag[enc_id] = true;
      break;
      // can_raw->power_Voltage
    case 0x210:  // m0
    case 0x211:  // m1
    case 0x212:  // m2
    case 0x213:  // m3
    case 0x214:  // sub board (not used)
    case 0x215:  // power battery
    case 0x216:  // power capacitor
      can_raw->power_voltage[rx_can_id - 0x210] = uchar4_to_float(rx_data);
      if (rx_can_id == 0x215) {
        can_raw->rx_stat.timeout_cnt[BOARD_ID_POWER] = 0;
      } else if (rx_can_id == 0x214) {
        can_raw->rx_stat.timeout_cnt[BOARD_ID_SUB] = 0;
      }
      break;

    // can_raw->temp_motor[],temp_driver[] from BLDC, motor & FET
    case 0x220:
    case 0x221:
    case 0x222:
    case 0x223:
      can_raw->temp_motor[rx_can_id - 0x220] = uchar4_to_float(rx_data);
      can_raw->temp_driver[rx_can_id - 0x220] = uchar4_to_float(&(rx_data[4]));
      break;
    // can_raw->temp_fet,temp_coil[] from power
    case 0x224:
      can_raw->temp_fet = rx_data[0];      // fet
      can_raw->temp_coil[0] = rx_data[1];  // coil 1
      can_raw->temp_coil[1] = rx_data[2];  // coil 2
      break;

    // can_raw->current
    case 0x230:
    case 0x231:
    case 0x232:
    case 0x233:
    case 0x234:
      can_raw->current[rx_can_id - 0x230] = uchar4_to_float(rx_data);
      break;

    // can_raw->ball_detection
    case 0x240:
      can_raw->ball_detection[0] = rx_data[0];
      can_raw->ball_detection[1] = rx_data[1];
      break;

    // mouseXY
    case 0x241:
      mouse->raw[0] = (int16_t)((rx_data[1] << 8) | rx_data[0]);
      mouse->raw[1] = (int16_t)((rx_data[3] << 8) | rx_data[2]);
      mouse->quality = (uint16_t)((rx_data[5] << 8) | rx_data[4]);
      can_raw->rx_stat.mouse_flag = true;

      // 持ち上げ･コート外検知
      /*if (mouse->quality < 30 && sys->system_time_ms > 1000) {
          error_flag = true;
        }*/
      break;

    case 0x500:  // モーターパラメーター
    case 0x501:
    case 0x502:
    case 0x503:
      can_raw->motor_param_rps[rx_can_id - 0x500] = uchar4_to_float(rx_data);
      break;
  }
}

static void sendKickCmd(RobotCommandV2 * ai_cmd, system_t * sys, can_raw_t * can_raw)
{
  if (ai_cmd->kick_power <= 0) {
    return;
  }

  if (sys->kick_state == 0) {
    if (can_raw->ball_detection[0] == 0) {
      return;
    }

    //start kick

    kicker_kick_start(ai_cmd->kick_power);

    sys->kick_state = 1;

  } else {
    // 0.5s以上待ち、ボールセンサーがクリアされるまでキックできないようにする
    if (sys->kick_state > 0.5 * MAIN_LOOP_CYCLE) {
      // end kick
      if (can_raw->ball_detection[0] == 0) {
        sys->kick_state = 0;
      }
    } else {
      sys->kick_state++;
    }
  }
}

static float getDribblerPower(RobotCommandV2 * ai_cmd, can_raw_t * can_raw)
{
  float dribbler_power = ai_cmd->dribble_power;
  if (!can_raw->ball_detection[0]) {
    dribbler_power /= 2;
  }
  return dribbler_power;
}

void sendActuatorCanCmdRun(RobotCommandV2 * ai_cmd, system_t * sys, can_raw_t * can_raw)
{
  sendKickCmd(ai_cmd, sys, can_raw);

  static uint8_t can_sending_index = 0;

  can_sending_index++;
  switch (can_sending_index) {
    case 1:
      if (ai_cmd->enable_chip == true) {
        kicker_select_chip();
      } else {
        kicker_select_straight();
      }
      break;

    case 2:
      kicker_charge_start();
      break;

    case 3:
      actuator_kicker_cmd_voltage(350.0);
      break;

    case 4:
      actuator_motor_drv(getDribblerPower(ai_cmd, can_raw));
      break;

    default:
      can_sending_index = 0;
      break;
  }
}

void sendActuatorCanCmdStop()
{
  actuator_motor_drv(0.0);

  kicker_charge_stop();
  actuator_kicker_cmd_voltage(0.0);
}

void sendCanError()
{
  uint8_t senddata_error[8] = {0};
  can1_send(0x000, senddata_error);
  can2_send(0x000, senddata_error);
  setHighEventLED();
}

void resetPowerBoard()
{
  uint8_t reset_power[8] = {0};
  reset_power[0] = 0x5A;
  reset_power[1] = 0xA5;
  can1_send(0x001, reset_power);
  can2_send(0x001, reset_power);
}

const int CAN_ENC_TIMEOUT_CYCLE_CNT = 100;

bool canRxTimeoutDetection(can_raw_t * can_raw)
{
  for (int i = 0; i < BOARD_ID_MAX; i++) {
    if (can_raw->rx_stat.timeout_cnt[i] > CAN_ENC_TIMEOUT_CYCLE_CNT) {
      return true;
    }
  }
  return false;
}

void canRxTimeoutCntCycle(can_raw_t * can_raw)
{
  for (int i = 0; i < BOARD_ID_MAX; i++) {
    if (can_raw->rx_stat.timeout_cnt[i] < CAN_ENC_TIMEOUT_CYCLE_CNT * 10) {
      can_raw->rx_stat.timeout_cnt[i]++;
    }
  }
}

// エンコーダ&オムニ角度取得済みかどうか
bool allEncInitialized(can_raw_t * can_raw)
{
  return can_raw->rx_stat.mouse_flag & can_raw->rx_stat.enc_flag[0] & can_raw->rx_stat.enc_flag[1] & can_raw->rx_stat.enc_flag[2] & can_raw->rx_stat.enc_flag[3];
}
