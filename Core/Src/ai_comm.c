#include "ai_comm.h"

#include "main.h"
#include "robot_packet.h"
#include "stop_state_control.h"
#include "util.h"

#define AI_CMD_TIMEOUT (0.5)
#define CM4_CMD_TIMEOUT (AI_CMD_TIMEOUT + 0.5)

#define TX_VALUE_ARRAY_SIZE (14)

// 以下float array

static int enqueueFloatArray(float array[], int idx, float data)
{
  if (idx >= TX_VALUE_ARRAY_SIZE || idx < 0) {
    return;
  }
  array[idx] = data;
  return idx + 1;
}

void sendRobotInfo(
  can_raw_t * can_raw, system_t * sys, imu_t * imu, omni_t * omni, mouse_t * mouse, RobotCommandV2 * ai_cmd, connection_t * con, integ_control_t * integ, output_t * out, target_t * target,
  camera_t * cam)
{
  static uint8_t buf[128];  // DMAで使用するためstaticでなければならない

  buf[0] = 0xAB;
  buf[1] = 0xEA;
  buf[2] = 10;
  buf[3] = ai_cmd->check_counter;

  float_to_uchar4(&(buf[4]), imu->yaw_deg);

  // battery(BLDC right)
  float_to_uchar4(&(buf[8]), can_raw->power_voltage[0]);

  buf[12] = can_raw->ball_detection[0];
  buf[13] = can_raw->ball_detection[1];
  buf[14] = can_raw->ball_detection[2];
  buf[15] = sys->kick_state / 10;

  buf[16] = (uint8_t)(sys->current_error.id & 0xFF);
  buf[17] = (uint8_t)((sys->current_error.id >> 8) & 0xFF);
  buf[18] = (uint8_t)(sys->current_error.info & 0xFF);
  buf[19] = (uint8_t)((sys->current_error.info >> 8) & 0xFF);

  float_to_uchar4(&(buf[20]), sys->current_error.value);

  buf[24] = (uint8_t)(can_raw->current[0] * 10);
  buf[25] = (uint8_t)(can_raw->current[1] * 10);
  buf[26] = (uint8_t)(can_raw->current[2] * 10);
  buf[27] = (uint8_t)(can_raw->current[3] * 10);

  buf[28] = can_raw->ball_detection[3];

  buf[29] = (uint8_t)can_raw->temp_motor[0];
  buf[30] = (uint8_t)can_raw->temp_motor[1];
  buf[31] = (uint8_t)can_raw->temp_motor[2];
  buf[32] = (uint8_t)can_raw->temp_motor[3];
  buf[33] = (uint8_t)can_raw->temp_fet;
  buf[34] = (uint8_t)can_raw->temp_coil[0];
  buf[35] = (uint8_t)can_raw->temp_coil[1];

  float diff_angle = imu->yaw_deg - ai_cmd->vision_global_theta;

  float_to_uchar4(&(buf[36]), diff_angle);
  // capacitor boost
  float_to_uchar4(&(buf[40]), can_raw->power_voltage[6]);

  float_to_uchar4(&(buf[44]), integ->vision_based_position[0]);
  float_to_uchar4(&(buf[48]), integ->vision_based_position[1]);

  float_to_uchar4(&(buf[52]), omni->global_odom_speed[0]);
  float_to_uchar4(&(buf[56]), omni->global_odom_speed[1]);

  buf[60] = cam->pos_xy[0] / 2;  // 0~340 -> 0-170
  buf[61] = cam->pos_xy[1];      // 0~180 -> 0-90
  buf[62] = cam->radius / 4;     // ??? -> ???
  buf[63] = cam->fps;            // ~60

  static float tx_value_array[TX_VALUE_ARRAY_SIZE] = {0};
  int value_idx = 0;
  value_idx = enqueueFloatArray(tx_value_array, value_idx, mouse->odom[0]);
  value_idx = enqueueFloatArray(tx_value_array, value_idx, mouse->odom[1]);

  value_idx = enqueueFloatArray(tx_value_array, value_idx, mouse->global_vel[0]);
  value_idx = enqueueFloatArray(tx_value_array, value_idx, mouse->global_vel[1]);

  value_idx = enqueueFloatArray(tx_value_array, value_idx, out->velocity[0]);
  value_idx = enqueueFloatArray(tx_value_array, value_idx, out->velocity[1]);

  value_idx = enqueueFloatArray(tx_value_array, value_idx, can_raw->motor_feedback[0]);
  value_idx = enqueueFloatArray(tx_value_array, value_idx, can_raw->motor_feedback[1]);

  value_idx = enqueueFloatArray(tx_value_array, value_idx, can_raw->motor_feedback[2]);
  value_idx = enqueueFloatArray(tx_value_array, value_idx, can_raw->motor_feedback[3]);

  //value_idx = enqueueFloatArray(tx_value_array, value_idx, target->global_vel_now[0]);
  //value_idx = enqueueFloatArray(tx_value_array, value_idx, target->global_vel_now[1]);

  value_idx = enqueueFloatArray(tx_value_array, value_idx, omni->local_odom_speed_mvf[0]);
  value_idx = enqueueFloatArray(tx_value_array, value_idx, omni->local_odom_speed_mvf[1]);
  value_idx = enqueueFloatArray(tx_value_array, value_idx, omni->local_odom_speed_mvf[2]);

  value_idx = enqueueFloatArray(tx_value_array, value_idx, mouse->quality);

  for (int i = 0; i < TX_VALUE_ARRAY_SIZE; i++) {
    float_to_uchar4(&(buf[64 + i * 4]), tx_value_array[i]);
  }

  HAL_UART_Transmit_DMA(&huart2, buf, sizeof(buf));
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
    if (self_timeout_reset_cnt > MAIN_LOOP_CYCLE * 3) {  // <- リセット時間
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