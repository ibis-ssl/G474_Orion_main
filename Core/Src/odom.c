/*
 * odom.c
 *
 *  Created on: Nov 17, 2023
 *      Author: hiroyuki
 */

#include "management.h"
#include "ring_buffer.h"
#include "robot_packet.h"
#include "util.h"

// 20ms cycle
void mouseOdometryUpdate(mouse_t * mouse, imu_t * imu)
{
  mouse->pre_odom[0] = mouse->odom[0];
  mouse->pre_odom[1] = mouse->odom[1];

  const int MOUSE_ODOM_CYCLE = 20;
  mouse->raw_diff[0] = -(float)mouse->raw[0] / MOUSE_ODOM_CYCLE;
  mouse->raw_diff[1] = -(float)mouse->raw[1] / MOUSE_ODOM_CYCLE;

  mouse->raw_odom[0] += mouse->raw_diff[0];
  mouse->raw_odom[1] += mouse->raw_diff[1];

  float floor_odom_diff[2] = {0, 0};
  convertLocalToGlobal(mouse->raw_diff, floor_odom_diff, imu->yaw_angle_rad);
  mouse->floor_odom[0] += floor_odom_diff[0];
  mouse->floor_odom[1] += floor_odom_diff[1];

  // 旋回速度ぶん補正
  const float DIST_CORRECITON_RATE = (0.066 / 1.56) / 3;  // No.5
  //  const float DIST_CORRECITON_RATE = 0.066 / 1.56;  // No.
  mouse->odom[0] = DIST_CORRECITON_RATE * mouse->floor_odom[0] - (DIST_CORRECITON_RATE * cos(imu->yaw_angle_rad) - DIST_CORRECITON_RATE);
  // X位置補正は誤差に埋もれてしまう。パラメーター調整を省略するために無効化
  //  +(0.009 * sin(imu->yaw_angle_rad));
  mouse->odom[1] = DIST_CORRECITON_RATE * mouse->floor_odom[1] - (DIST_CORRECITON_RATE * sin(imu->yaw_angle_rad));
  //  +(0.009 * cos(imu->yaw_angle_rad) - 0.009);

  mouse->global_vel[0] = mouse->odom[0] - mouse->pre_odom[0];
  mouse->global_vel[1] = mouse->odom[1] - mouse->pre_odom[1];

  mouse->pre_yaw_angle_rad = imu->yaw_angle_rad;
}

void omniOdometryUpdate(motor_t * motor, omni_t * omni, imu_t * imu)
{
  // motor->enc_angle,/imu->yaw_angle_rad

  for (int i = 0; i < 4; i++) {
    if (isnan(motor->enc_angle[i])) {
      motor->enc_angle[i] = 0;
    }
    motor->angle_diff[i] = getAngleDiff(motor->pre_enc_angle[i], motor->enc_angle[i]);
    motor->pre_enc_angle[i] = motor->enc_angle[i];
    omni->travel_distance[i] = motor->angle_diff[i] * OMNI_DIAMETER;
  }

  // right back & left back

  // 右後方車輪を基準とした座標系､travel_distance[2]は反転している
  float raw_odom_angle = -(M_PI * 1 / 4);  // + imu->yaw_angle_rad;
  omni->local_raw_odom_vel[0] = omni->travel_distance[1] * cos(raw_odom_angle) + omni->travel_distance[2] * sin(raw_odom_angle);
  omni->local_raw_odom_vel[1] = omni->travel_distance[1] * sin(raw_odom_angle) - omni->travel_distance[2] * cos(raw_odom_angle);

  omni->local_raw_odom_vel[0] *= MAIN_LOOP_CYCLE / 2;
  omni->local_raw_odom_vel[1] *= MAIN_LOOP_CYCLE / 2;

  float global_raw_odom_vel[2] = {0, 0};
  convertLocalToGlobal(omni->local_raw_odom_vel, global_raw_odom_vel, imu->yaw_angle_rad);

  // 後輪2輪によるodom
  // 基準座標の原点が後輪輪の推力線の交点になるので､機体中心位置までずらす
  // 原点座標が交点にあるため､回転の影響を無視できる
  float zero_point_offset[2] = {-0.107, 0};  //, offset_dist[2];
  convertLocalToGlobal(zero_point_offset, omni->offset_dist, imu->yaw_angle_rad);

  for (int i = 0; i < 2; i++) {
    omni->global_raw_odom[i] += global_raw_odom_vel[i] / MAIN_LOOP_CYCLE;

    omni->pre_odom[i] = omni->odom[i];

    omni->odom[i] = omni->global_raw_odom[i] - omni->offset_dist[i];

    omni->global_odom_speed[i] = (omni->odom[i] - omni->pre_odom[i]) * MAIN_LOOP_CYCLE;
  }
  //
  convertGlobalToLocal(omni->global_odom_speed, omni->local_odom_speed, imu->yaw_angle_rad);

  omni->local_odom_speed[2] = (omni->travel_distance[0] - omni->travel_distance[3]) * MAIN_LOOP_CYCLE * 0.5 * 0.5;

  for (int i = 0; i < 3; i++) {
    enqueue(omni->local_speed_log[i], omni->local_odom_speed[i]);
    omni->local_odom_speed_mvf[i] = sumNewestN(omni->local_speed_log[i], SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE) / SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE;
  }
}

void inntegOdomUpdate(RobotCommandV2 * ai_cmd, omni_t * omni, integration_control_t * integ, connection_t * connection, imu_t * imu, system_t * sys)
{
  connection->ai_cmd_delta_time = sys->system_time_ms - connection->latest_ai_cmd_update_time;
  float latency_cycle = (ai_cmd->latency_time_ms + ai_cmd->elapsed_time_ms_since_last_vision + connection->ai_cmd_delta_time) / (1000 / MAIN_LOOP_CYCLE);
  for (int i = 0; i < 2; i++) {
    enqueue(integ->odom_log[i], omni->global_odom_speed[i]);
    integ->global_odom_vision_diff[i] = sumNewestN(integ->odom_log[i], latency_cycle) / MAIN_LOOP_CYCLE;
    integ->vision_based_position[i] = ai_cmd->vision_global_pos[i] + integ->global_odom_vision_diff[i];
  }
}