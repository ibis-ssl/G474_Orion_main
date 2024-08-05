#include "ai_comm.h"

#include "robot_packet.h"
#include "stop_state_control.h"
#include "util.h"
#define MAX_AI_CMD_SPEED_SCALAR_LIMIT (6.0)

#define AI_CMD_TIMEOUT (0.5)
#define CM4_CMD_TIMEOUT (AI_CMD_TIMEOUT + 0.5)

void sendRobotInfo(
  can_raw_t * can_raw, system_t * sys, imu_t * imu, omni_t * omni, mouse_t * mouse, RobotCommandV2 * ai_cmd, connection_t * con, integration_control_t * integ, output_t * out, target_t * target)
{
  static uint8_t buf[128];  // DMAで使用するためstaticでなければならない

  buf[0] = 0xAB;
  buf[1] = 0xEA;
  buf[2] = 10;
  buf[3] = ai_cmd->check_counter;

  float_to_uchar4(&(buf[4]), imu->yaw_angle);

  // battery(from sub)
  float_to_uchar4(&(buf[8]), can_raw->power_voltage[4]);

  buf[12] = can_raw->ball_detection[0];
  buf[13] = can_raw->ball_detection[1];
  buf[14] = can_raw->ball_detection[2];
  buf[15] = sys->kick_state / 10;

  buf[16] = (uint8_t)(sys->error_id & 0xFF);
  buf[17] = (uint8_t)((sys->error_id >> 8) & 0xFF);
  buf[18] = (uint8_t)(sys->error_info & 0xFF);
  buf[19] = (uint8_t)((sys->error_info >> 8) & 0xFF);

  float_to_uchar4(&(buf[20]), sys->error_value);

  buf[24] = (uint8_t)(can_raw->current[0] * 10);
  buf[25] = (uint8_t)(can_raw->current[1] * 10);
  buf[26] = (uint8_t)(can_raw->current[2] * 10);
  buf[27] = (uint8_t)(can_raw->current[3] * 10);

  buf[28] = can_raw->ball_detection[3];

  buf[29] = (uint8_t)can_raw->temperature[0];
  buf[30] = (uint8_t)can_raw->temperature[1];
  buf[31] = (uint8_t)can_raw->temperature[2];
  buf[32] = (uint8_t)can_raw->temperature[3];
  buf[33] = (uint8_t)can_raw->temperature[4];
  buf[34] = (uint8_t)can_raw->temperature[5];
  buf[35] = (uint8_t)can_raw->temperature[6];

  float diff_angle = imu->yaw_angle - ai_cmd->vision_global_theta;

  float_to_uchar4(&(buf[36]), diff_angle);
  // capacitor boost
  float_to_uchar4(&(buf[40]), can_raw->power_voltage[6]);
  float_to_uchar4(&(buf[44]), omni->odom[0]);
  float_to_uchar4(&(buf[48]), omni->odom[1]);
  float_to_uchar4(&(buf[52]), omni->global_odom_speed[0]);
  float_to_uchar4(&(buf[56]), omni->global_odom_speed[1]);

  buf[60] = con->check_ver;
  buf[61] = 0;
  buf[62] = 0;
  buf[63] = 0;

  // 以下float array

  float_to_uchar4(&(buf[64]), mouse->odom[0]);
  float_to_uchar4(&(buf[68]), mouse->odom[1]);
  float_to_uchar4(&(buf[72]), mouse->global_vel[0]);
  float_to_uchar4(&(buf[76]), mouse->global_vel[1]);

  float_to_uchar4(&(buf[80]), integ->vision_based_position[0]);
  float_to_uchar4(&(buf[84]), integ->vision_based_position[1]);
  float_to_uchar4(&(buf[88]), out->velocity[0]);
  float_to_uchar4(&(buf[92]), out->velocity[1]);

  float_to_uchar4(&(buf[96]), out->accel[0]);
  float_to_uchar4(&(buf[100]), out->accel[1]);

  float_to_uchar4(&(buf[104]), target->global_vel_now[0]);
  float_to_uchar4(&(buf[108]), target->global_vel_now[1]);
  float mouse_q = mouse->quality;
  float_to_uchar4(&(buf[112]), mouse_q);

  float_to_uchar4(&(buf[116]), omni->local_odom_speed_mvf[0]);
  float_to_uchar4(&(buf[120]), omni->local_odom_speed_mvf[1]);
  float_to_uchar4(&(buf[124]), omni->local_odom_speed_mvf[2]);

  HAL_UART_Transmit_DMA(&huart2, buf, sizeof(buf));
}

static void updateAICmdTimeStamp(connection_t * connection, system_t * sys) { connection->latest_ai_cmd_update_time = sys->system_time_ms; }
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
  ai_cmd->omega_limit = 0;
  ai_cmd->speed_limit = 0;
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

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

  } else {
    connection->connected_ai = false;
    connection->cmd_rx_frq = 0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    resetAiCmdData(ai_cmd);

    requestStop(sys, 1.0);
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
