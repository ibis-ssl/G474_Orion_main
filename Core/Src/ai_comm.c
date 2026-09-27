// CM4との制御指令受信と、G474の状態を128バイトfeedbackとして送信する処理を担当する。
#include "ai_comm.h"

#include "main.h"
#include "robot_packet.h"
#include "stop_state_control.h"
#include "util.h"
#include <string.h>

#define AI_CMD_TIMEOUT (0.5)
#define CM4_CMD_TIMEOUT (AI_CMD_TIMEOUT + 0.5)

// CRC-8/ATM: poly=0x07, init=0x00, refin/refout=false, xorout=0x00。
static uint8_t feedbackCrc8(const uint8_t * data, uint32_t size)
{
  uint8_t crc = 0;
  for (uint32_t i = 0; i < size; i++) {
    crc ^= data[i];
    for (uint32_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void sendRobotInfo(
  can_raw_t * can_raw, system_t * sys, imu_t * imu, omni_t * omni, mouse_t * mouse, RobotCommandV2 * ai_cmd, connection_t * con, integ_control_t * integ, output_t * out, target_t * target)
{
  static uint8_t buf[128];  // DMA送信が完了するまで保持する
  static uint8_t tx_cycle_count = 0;

  (void)con;
  (void)target;
  if (huart2.gState != HAL_UART_STATE_READY) return;
  memset(buf, 0, sizeof(buf));

  buf[0] = 0xAB;
  buf[1] = 0xEA;
  buf[3] = ai_cmd->check_counter;
  buf[4] = tx_cycle_count;
  buf[5] = (uint8_t)(sys->current_error.id & 0xFF);
  buf[6] = (uint8_t)((sys->current_error.id >> 8) & 0xFF);
  buf[7] = (uint8_t)(sys->current_error.info & 0xFF);
  buf[8] = (uint8_t)((sys->current_error.info >> 8) & 0xFF);
  float_to_uchar4(&buf[9], sys->current_error.value);

  float_to_uchar4(&buf[13], imu->yaw_deg);
  buf[17] = can_raw->ball_detection[0];
  buf[18] = can_raw->ball_detection[1];
  buf[19] = 0;  // 追加のボール検出値はOrionMainでは未使用
  float_to_uchar4(&buf[20], imu->yaw_deg - radian_to_deg(ai_cmd->vision_global_theta));

  float_to_uchar4(&buf[24], can_raw->power_voltage[0]);
  buf[28] = sys->kick_state / 10;
  buf[29] = (uint8_t)can_raw->temp_fet;
  buf[30] = (uint8_t)can_raw->temp_coil[0];
  buf[31] = (uint8_t)can_raw->temp_coil[1];
  float_to_uchar4(&buf[32], can_raw->power_voltage[6]);
  float_to_uchar4(&buf[36], mouse->odom[0]);
  float_to_uchar4(&buf[40], mouse->odom[1]);
  float_to_uchar4(&buf[44], mouse->global_vel[0]);
  float_to_uchar4(&buf[48], mouse->global_vel[1]);
  float_to_uchar4(&buf[52], mouse->quality);

  for (uint32_t i = 0; i < 4U; i++) {
    buf[56U + i] = (uint8_t)(can_raw->current[i] * 10);
    buf[60U + i] = (uint8_t)can_raw->temp_motor[i];
    float_to_uchar4(&buf[72U + 4U * i], can_raw->motor_feedback[i]);
    buf[100U + 2U * i] = 0x7F;  // Orionにはステアがないため角度0の符号化値
    buf[101U + 2U * i] = 0xFF;
  }
  float_to_uchar4(&buf[64], out->velocity[0]);
  float_to_uchar4(&buf[68], out->velocity[1]);
  for (uint32_t i = 0; i < 3U; i++) {
    float_to_uchar4(&buf[88U + 4U * i], omni->local_odom_speed_mvf[i]);
  }
  float_to_uchar4(&buf[112], integ->vision_based_position[0]);
  float_to_uchar4(&buf[116], integ->vision_based_position[1]);
  float_to_uchar4(&buf[120], omni->global_odom_speed[0]);
  float_to_uchar4(&buf[124], omni->global_odom_speed[1]);

  buf[2] = feedbackCrc8(&buf[3], sizeof(buf) - 3U);

  if (HAL_UART_Transmit_DMA(&huart2, buf, sizeof(buf)) == HAL_OK) tx_cycle_count++;
}

static void updateAICmdTimeStamp(connection_t * connection, system_t * sys)
{
  connection->ai_cmd_rx_cnt++;
  connection->latest_ai_cmd_update_time = sys->system_time_ms;
}
void updateCM4CmdTimeStamp(connection_t * connection, system_t * sys)
{
  connection->updated_flag = true;
  connection->latest_cm4_cmd_update_time = sys->system_time_ms;
}

static void checkConnect2CM4(connection_t * connection, system_t * sys)
{
  // CM4との通信状態チェック
  if (sys->system_time_ms - connection->latest_cm4_cmd_update_time < MAIN_LOOP_CYCLE * CM4_CMD_TIMEOUT) {  // CM4 コマンドタイムアウト
    connection->connected_cm4 = true;
  } else {
    connection->connected_cm4 = false;
    connection->connected_ai = false;
  }
}

void resetAiCmdData(RobotCommandV2 * ai_cmd)
{
  ai_cmd->dribble_power = 0;
  ai_cmd->is_vision_available = false;
  ai_cmd->stop_emergency = true;
  ai_cmd->angular_velocity_limit = 0;
  ai_cmd->linear_velocity_limit = 0;
  ai_cmd->kick_power = 0;
  ai_cmd->enable_chip = false;
  ai_cmd->dribble_power = 0;
}

static void checkConnect2AI(connection_t * connection, system_t * sys, RobotCommandV2 * ai_cmd)
{
  static uint8_t pre_ai_counter = 0;
  if (pre_ai_counter != ai_cmd->check_counter) {
    pre_ai_counter = ai_cmd->check_counter;
    updateAICmdTimeStamp(connection, sys);
  }

  // AIとの通信状態チェック
  if (sys->system_time_ms - connection->latest_ai_cmd_update_time < MAIN_LOOP_CYCLE * AI_CMD_TIMEOUT) {  // AI コマンドタイムアウト
    connection->connected_ai = true;
    connection->already_connected_ai = true;

    setHighUartRxLED();
  } else {
    connection->connected_ai = false;
    connection->ai_cmd_rx_frq = 0;
    setLowUartRxLED();
    resetAiCmdData(ai_cmd);

    requestStop(sys, 1000);
  }
}

// "一度はAI側から接続があったあと"､CM4との通信が途切れたらリセット
// JO2024でたまに動作中にマイコンのUART受信が止まることがあったので､対策として導入
static bool disconnedtedFromCM4(connection_t * connection, system_t * sys)
{
  if (sys->main_mode == MAIN_MODE_CMD_DEBUG_MODE) {
    return false;
  }
  if (!connection->connected_cm4 && connection->already_connected_ai) {
    return true;
  }
  return false;
}

void commStateCheck(connection_t * connection, system_t * sys, RobotCommandV2 * ai_cmd)
{
  checkConnect2CM4(connection, sys);
  checkConnect2AI(connection, sys, ai_cmd);

  // AI通信切断時、3sでリセット
  static uint32_t self_timeout_reset_cnt = 0;
  if (disconnedtedFromCM4(connection, sys)) {
    self_timeout_reset_cnt++;
    if (self_timeout_reset_cnt > MAIN_LOOP_CYCLE * 6) {  // <- リセット時間
      NVIC_SystemReset();
    }
  } else {
    self_timeout_reset_cnt = 0;
  }
}

camera_t parseCameraPacket(uint8_t * data)
{
  camera_t camera;
  // x = [x_high, x_low, y_high, y_low, radius_high, radius_low, FPS_send]
  camera.pos_xy[0] = (data[0] << 8) + data[1];
  camera.pos_xy[1] = (data[2] << 8) + data[3];
  camera.radius = (data[4] << 8) + data[5];
  camera.fps = data[6];
  return camera;
}

static uint8_t calcCheckSum(uint8_t data[])
{
  uint32_t rx_check_cnt_all = 0;

  // 最終byteがcheckcntなので除外する
  for (int i = 0; i < RX_BUF_SIZE_CM4 - 1; i++) {
    rx_check_cnt_all += data[i];
  }

  return rx_check_cnt_all & 0xFF;
}

bool checkCM4CmdCheckSun(connection_t * connection, uint8_t data[])
{
  connection->check_cnt = calcCheckSum(data);
  if (connection->check_cnt != data[RX_BUF_SIZE_CM4 - 1]) {
    connection->check_sum_error_cnt++;
    return false;
  } else {
    return true;
  }
}
