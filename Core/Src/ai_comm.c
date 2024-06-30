#include "ai_comm.h"

#include "util.h"

#define MAX_AI_CMD_SPEED_SCALAR_LIMIT (6.0)

void resetAiCmdData(ai_cmd_t * ai_cmd)
{
  ai_cmd->local_target_speed[0] = 0;
  ai_cmd->local_target_speed[1] = 0;
  ai_cmd->global_vision_theta = 0;
  ai_cmd->target_theta = 0;
  ai_cmd->chip_en = false;
  ai_cmd->kick_power = 0;
  ai_cmd->dribble_power = 0;
  ai_cmd->allow_local_flags = 0;

  ai_cmd->global_ball_position[0] = 0;
  ai_cmd->global_ball_position[1] = 0;
  ai_cmd->global_robot_position[0] = 0;
  ai_cmd->global_robot_position[1] = 0;
  ai_cmd->global_target_position[0] = 0;
  ai_cmd->global_target_position[1] = 0;

  // ローカルカメラ情報は消さない(デバッグ用)
  /*
  ai_cmd->ball_local_x = 0;
  ai_cmd->ball_local_y = 0;
  ai_cmd->ball_local_radius = 0;
  ai_cmd->ball_local_FPS = 0;*/

  ai_cmd->vision_lost_flag = true;
  ai_cmd->local_vision_en_flag = false;
  ai_cmd->keeper_mode_en_flag = false;
  ai_cmd->stop_request_flag = false;  //
}

void parseRxCmd(connection_t * con, system_t * sys, ai_cmd_t * ai_cmd, integration_control_t * integ, uint8_t data[])
{
  con->check_ver = data[1];

  if (con->check_ver != con->check_pre) {
    con->latest_ai_cmd_update_time = sys->system_time_ms;

    con->check_pre = con->check_ver;
  }

  float pre_update_time_ms = con->latest_cm4_cmd_update_time;
  con->latest_cm4_cmd_update_time = sys->system_time_ms;
  con->cmd_rx_frq = (float)1000 / (con->latest_cm4_cmd_update_time - pre_update_time_ms);

  // aiコマンドに関係なくカメラ情報は入れる(デバッグ用)
  ai_cmd->ball_local_x = data[RX_BUF_SIZE_ETHER - 7] << 8 | data[RX_BUF_SIZE_ETHER - 6];
  ai_cmd->ball_local_y = data[RX_BUF_SIZE_ETHER - 5] << 8 | data[RX_BUF_SIZE_ETHER - 4];
  ai_cmd->ball_local_radius = data[RX_BUF_SIZE_ETHER - 3] << 8 | data[RX_BUF_SIZE_ETHER - 2];
  ai_cmd->ball_local_FPS = data[RX_BUF_SIZE_ETHER - 1];

  // timer割り込み側でtimeout検知
  // バッファクリア→timer割り込み側でクリアする
  if (con->connected_ai == false) {
    resetAiCmdData(ai_cmd);
    con->updated_flag = true;
    return;
  }

  ai_cmd->local_target_speed[0] = two_to_float(&data[2]) * AI_CMD_VEL_MAX_MPS;
  ai_cmd->local_target_speed[1] = two_to_float(&data[4]) * AI_CMD_VEL_MAX_MPS;

  ai_cmd->local_target_speed_scalar = pow(pow(ai_cmd->local_target_speed[0], 2) + pow(ai_cmd->local_target_speed[1], 2), 0.5);
  /*if (ai_cmd->local_target_speed_scalar > MAX_AI_CMD_SPEED_SCALAR_LIMIT) {
    ai_cmd->local_target_speed[0] *= MAX_AI_CMD_SPEED_SCALAR_LIMIT / ai_cmd->local_target_speed_scalar;
    ai_cmd->local_target_speed[1] *= MAX_AI_CMD_SPEED_SCALAR_LIMIT / ai_cmd->local_target_speed_scalar;
  }*/

  ai_cmd->global_vision_theta = two_to_float(&data[6]) * M_PI;
  ai_cmd->target_theta = two_to_float(&data[8]) * M_PI;
  if (data[10] >= 101) {
    ai_cmd->chip_en = true;
    ai_cmd->kick_power = (float)(data[10] - 101) / 20;
  } else {
    ai_cmd->kick_power = (float)data[10] / 20;
    ai_cmd->chip_en = false;
  }
  ai_cmd->dribble_power = (float)data[11] / 20;

  ai_cmd->allow_local_flags = data[12];

  // integとai_cmdで分けてるだけで同じ情報の now と pre
  integ->pre_global_target_position[0] = ai_cmd->global_target_position[0];
  integ->pre_global_target_position[1] = ai_cmd->global_target_position[1];

  // <int>[mm] -> <float>[m]
  ai_cmd->global_ball_position[0] = (float)two_to_int(&data[13]) / 1000;
  ai_cmd->global_ball_position[1] = (float)two_to_int(&data[15]) / 1000;
  ai_cmd->global_robot_position[0] = (float)two_to_int(&data[17]) / 1000;
  ai_cmd->global_robot_position[1] = (float)two_to_int(&data[19]) / 1000;
  ai_cmd->global_target_position[0] = (float)two_to_int(&data[21]) / 1000;
  ai_cmd->global_target_position[1] = (float)two_to_int(&data[23]) / 1000;

  // 値がおかしい時は0にする (+-30を超えることはない)
  for (int i = 0; i < 2; i++) {
    if (ai_cmd->global_target_position[i] > 30.0 || ai_cmd->global_target_position[i] < -30) {
      ai_cmd->global_target_position[i] = 0;
    }
  }

  if ((ai_cmd->allow_local_flags & FLAG_SSL_VISION_OK) != 0) {
    ai_cmd->vision_lost_flag = false;
  } else {
    ai_cmd->vision_lost_flag = true;
  }
  if (!ai_cmd->vision_lost_flag) {
    con->vision_update_cycle_cnt = 0;
  }

  if ((ai_cmd->allow_local_flags & FLAG_ENABLE_LOCAL_VISION) != 0) {
    ai_cmd->local_vision_en_flag = true;
  } else {
    ai_cmd->local_vision_en_flag = false;
  }

  if ((ai_cmd->allow_local_flags & FLAG_ENABLE_KEEPER_MODE) != 0) {
    ai_cmd->keeper_mode_en_flag = true;
  } else {
    ai_cmd->keeper_mode_en_flag = false;
  }

  if ((ai_cmd->allow_local_flags & FLAG_STOP_REQUEST) != 0) {
    ai_cmd->stop_request_flag = true;
  } else {
    ai_cmd->stop_request_flag = false;
  }

  if ((ai_cmd->allow_local_flags & FLAG_DRIBBLER_UP) != 0) {
    ai_cmd->dribbler_up_flag = true;
  } else {
    ai_cmd->dribbler_up_flag = false;
  }

  con->updated_flag = true;
}

void sendRobotInfo(can_raw_t * can_raw, system_t * sys, imu_t * imu, omni_t * omni, mouse_t * mouse, ai_cmd_t * ai_cmd, connection_t * con)
{
  tx_msg_t msg;
  static uint8_t ring_counter = 0;

  ring_counter++;
  if (ring_counter > 200) {
    ring_counter = 0;
  }

  // AI側のcheckをそのまま返す
  ring_counter = con->check_ver;
  char * temp;

  static uint8_t senddata[64];

  senddata[0] = 0xAB;
  senddata[1] = 0xEA;
  senddata[2] = 10;
  senddata[3] = ring_counter;
  temp = (char *)&imu->yaw_angle;
  senddata[4] = temp[0];
  senddata[5] = temp[1];
  senddata[6] = temp[2];
  senddata[7] = temp[3];
  temp = (char *)&(can_raw->power_voltage[5]);
  senddata[8] = temp[0];
  senddata[9] = temp[1];
  senddata[10] = temp[2];
  senddata[11] = temp[3];
  senddata[12] = can_raw->ball_detection[0];
  senddata[13] = can_raw->ball_detection[1];
  senddata[14] = can_raw->ball_detection[2];
  senddata[15] = sys->kick_state / 10;

  senddata[16] = (uint8_t)(sys->error_id & 0xFF);
  senddata[17] = (uint8_t)((sys->error_id >> 8) & 0xFF);
  senddata[18] = (uint8_t)(sys->error_info & 0xFF);
  senddata[19] = (uint8_t)((sys->error_info >> 8) & 0xFF);
  temp = (char *)&(sys->error_value);
  senddata[20] = temp[0];
  senddata[21] = temp[1];
  senddata[22] = temp[2];
  senddata[23] = temp[3];

  senddata[24] = (uint8_t)(can_raw->current[0] * 10);
  senddata[25] = (uint8_t)(can_raw->current[1] * 10);
  senddata[26] = (uint8_t)(can_raw->current[2] * 10);
  senddata[27] = (uint8_t)(can_raw->current[3] * 10);

  senddata[28] = can_raw->ball_detection[3];

  senddata[29] = (uint8_t)can_raw->temperature[0];
  senddata[30] = (uint8_t)can_raw->temperature[1];
  senddata[31] = (uint8_t)can_raw->temperature[2];
  senddata[32] = (uint8_t)can_raw->temperature[3];
  senddata[33] = (uint8_t)can_raw->temperature[4];
  senddata[34] = (uint8_t)can_raw->temperature[5];
  senddata[35] = (uint8_t)can_raw->temperature[6];

  msg.data.diff_angle = imu->yaw_angle - ai_cmd->global_vision_theta;
  temp = (char *)&msg.data.diff_angle;
  senddata[36] = temp[0];
  senddata[37] = temp[1];
  senddata[38] = temp[2];
  senddata[39] = temp[3];
  temp = (char *)&(can_raw->power_voltage[6]);
  senddata[40] = temp[0];
  senddata[41] = temp[1];
  senddata[42] = temp[2];
  senddata[43] = temp[3];
  temp = (char *)&omni->odom[0];
  senddata[44] = temp[0];
  senddata[45] = temp[1];
  senddata[46] = temp[2];
  senddata[47] = temp[3];
  temp = (char *)&omni->odom[1];
  senddata[48] = temp[0];
  senddata[49] = temp[1];
  senddata[50] = temp[2];
  senddata[51] = temp[3];
  temp = (char *)&omni->odom_speed[0];
  //temp = (char *)&integ.vision_based_position[0];
  senddata[52] = temp[0];
  senddata[53] = temp[1];
  senddata[54] = temp[2];
  senddata[55] = temp[3];
  temp = (char *)&omni->odom_speed[1];
  //temp = (char *)&integ.vision_based_position[1];
  senddata[56] = temp[0];
  senddata[57] = temp[1];
  senddata[58] = temp[2];
  senddata[59] = temp[3];
  senddata[60] = con->check_ver;
  senddata[61] = 0;
  senddata[62] = 0;
  senddata[63] = 0;

  HAL_UART_Transmit_DMA(&huart2, senddata, sizeof(senddata));
}