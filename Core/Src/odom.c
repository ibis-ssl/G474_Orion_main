/*
 * odom.c
 *
 *  Created on: Nov 17, 2023
 *      Author: hiroyuki
 */

#include "management.h"
#include "ring_buffer.h"
#include "util.h"

void convertGlobalToLocal(float global[], float local[], float yaw_rad)
{
  local[0] = global[0] * cos(-yaw_rad) - global[1] * sin(-yaw_rad);
  local[1] = global[0] * sin(-yaw_rad) + global[1] * cos(-yaw_rad);
}

void convertLocalToGlobal(float local[], float global[], float yaw_rad)
{
  global[0] = local[0] * cos(yaw_rad) - local[1] * sin(yaw_rad);
  global[1] = local[0] * sin(yaw_rad) + local[1] * cos(yaw_rad);
}

// 20ms cycle
void mouseOdometry(mouse_t * mouse, imu_t * imu)
{
  const int MOUSE_ODOM_CYCLE = 20;
  mouse->raw_diff[0] = (float)mouse->raw[0] / MOUSE_ODOM_CYCLE;
  mouse->raw_diff[1] = (float)mouse->raw[1] / MOUSE_ODOM_CYCLE;

  mouse->raw_odom[0] += mouse->raw_diff[0];
  mouse->raw_odom[1] += mouse->raw_diff[1];

  float floor_odom_diff[2] = {0, 0};
  convertLocalToGlobal(mouse->raw_diff, floor_odom_diff, imu->yaw_angle_rad);
  mouse->floor_odom[0] += floor_odom_diff[0];
  mouse->floor_odom[1] += floor_odom_diff[1];

  // 旋回ぶん補正
  const float DIST_CORRECITON_RATE = 0.066 / 3 * 1.56;
  mouse->odom[0] = DIST_CORRECITON_RATE * mouse->floor_odom[0] - (DIST_CORRECITON_RATE * cos(imu->yaw_angle_rad) - DIST_CORRECITON_RATE);
  // X位置補正は誤差に埋もれてしまう。パラメーター調整を省略するために無効化
  //  +(0.009 * sin(imu->yaw_angle_rad));
  mouse->odom[1] = DIST_CORRECITON_RATE * mouse->floor_odom[1] - (DIST_CORRECITON_RATE * sin(imu->yaw_angle_rad));
  //  +(0.009 * cos(imu->yaw_angle_rad) - 0.009);

  mouse->pre_yaw_angle_rad = imu->yaw_angle_rad;
}

void omniOdometry(ai_cmd_t * ai_cmd, motor_t * motor, omni_t * omni, integration_control_t * integ, connection_t * connection, imu_t * imu)
{
  // motor->enc_angle,/imu->yaw_angle_rad

  for (int i = 0; i < 4; i++) {
    if (isnan(motor->enc_angle[i])) {
      motor->enc_angle[i] = 0;
    }
    motor->angle_diff[i] = getAngleDiff(motor->enc_angle[i], motor->pre_enc_angle[i]);
    motor->pre_enc_angle[i] = motor->enc_angle[i];
  }

  // float robot_rotation_adj;
  // robot_rotation_adj = normalizeAngle(imu->yaw_angle_rad - imu->pre_yaw_angle_rad) * OMNI_ROTATION_LENGTH;  // mm

  omni->travel_distance[0] = motor->angle_diff[1] * OMNI_DIAMETER;
  //  +robot_rotation_adj * 2;
  omni->travel_distance[1] = motor->angle_diff[2] * OMNI_DIAMETER;
  //  +robot_rotation_adj * 2;

  // right back & left back
  float global_travel[2] = {0, 0};
  convertLocalToGlobal(omni->travel_distance, global_travel, imu->yaw_angle_rad);
  omni->odom_raw[0] += global_travel[0];
  omni->odom_raw[1] += global_travel[1];

  omni->pre_odom[0] = omni->odom[0];
  omni->pre_odom[1] = omni->odom[1];

  // 後輪2輪によるodom
  omni->odom[0] = ((omni->odom_raw[0] * cos(M_PI * 3 / 4) - omni->odom_raw[1] * sin(M_PI * 3 / 4)) / 2) + (0.107 * cos(imu->yaw_angle_rad) - 0.107);
  omni->odom[1] = ((omni->odom_raw[0] * sin(M_PI * 3 / 4) + omni->odom_raw[1] * cos(M_PI * 3 / 4)) / 2) + (0.107 * sin(imu->yaw_angle_rad));

  omni->odom_speed[0] = (omni->odom[0] - omni->pre_odom[0]) * MAIN_LOOP_CYCLE;
  omni->odom_speed[1] = (omni->odom[1] - omni->pre_odom[1]) * MAIN_LOOP_CYCLE;

  convertGlobalToLocal(omni->odom_speed, omni->local_odom_speed, imu->yaw_angle);
  for (int i = 0; i < 2; i++) {
    enqueue(omni->local_speed_log[i], omni->local_odom_speed[i]);
    omni->local_odom_speed_mvf[i] = sumNewestN(omni->local_speed_log[i], SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE) / SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE;
  }

  // local座標系で入れているodom speedを,global系に修正する
  // vision座標だけ更新されているが、vision_update_cycle_cntが0になっていない場合に、1cycleだけpositionが飛ぶ

  float latency_cycle = ai_cmd->latency_time_ms / (1000 / MAIN_LOOP_CYCLE);
  for (int i = 0; i < 2; i++) {
    enqueue(integ->odom_log[i], omni->odom_speed[i]);
    integ->global_odom_vision_diff[i] = sumNewestN(integ->odom_log[i], latency_cycle + connection->vision_update_cycle_cnt) / MAIN_LOOP_CYCLE;
    integ->vision_based_position[i] = ai_cmd->global_robot_position[i] + integ->global_odom_vision_diff[i];
    integ->position_diff[i] = ai_cmd->global_target_position[i] - integ->vision_based_position[i];
  }

  float target_diff[2], move_diff[2];
  for (int i = 0; i < 2; i++) {
    target_diff[i] = ai_cmd->global_robot_position[i] - ai_cmd->global_target_position[i];  // Visionが更新された時点での現在地とtargetの距離
    move_diff[i] = ai_cmd->global_robot_position[i] - integ->vision_based_position[i];      // Visionとtargetが更新されてからの移動量
  }

  integ->target_dist_diff = sqrt(pow(target_diff[0], 2) + pow(target_diff[1], 2));
  integ->move_dist = sqrt(pow(integ->position_diff[0], 2) + pow(integ->position_diff[1], 2));
}