/*
 * オムニホイールの目標角度生成、角度追従制御、静止時の角度誤差クリアを担当する。
 */
#include "control_omni_angle.h"

#include "control_theta.h"
#include "math.h"
#include "util.h"

#define ANGLE_CLEAR_LOCAL_VELOCITY_THRESHOLD_MPS (0.01f)
#define ANGLE_CLEAR_WHEEL_RPS_THRESHOLD (0.05f)
#define ANGLE_CLEAR_YAW_RATE_THRESHOLD_RAD_PER_SEC (0.05f)
#define ANGLE_CLEAR_CAN_RX_MAX_AGE_MS (4U)
#define ANGLE_CLEAR_STABLE_CYCLES (2U)  // 4 ms at 500 Hz
#define ANGLE_CLEAR_TIME_CONSTANT_SEC (0.05f)
#define ANGLE_CLEAR_ALPHA (1.0f / (ANGLE_CLEAR_TIME_CONSTANT_SEC * MAIN_LOOP_CYCLE + 1.0f))
#define ANGLE_CLEAR_MAX_RATE_RAD_PER_SEC (0.5f)
#define ANGLE_CLEAR_MAX_STEP_RAD (ANGLE_CLEAR_MAX_RATE_RAD_PER_SEC / MAIN_LOOP_CYCLE)
#define ANGLE_CLEAR_FINISH_THRESHOLD_RAD (0.0005f)

static const float rotation_length_omni = OMNI_DIAMETER * M_PI;
static const float sinM1 = sin(30 * M_PI / 180);
static const float cosM1 = cos(30 * M_PI / 180);

static const float sinM2 = sin(315 * M_PI / 180);
static const float cosM2 = cos(315 * M_PI / 180);

static const float sinM3 = sin(225 * M_PI / 180);
static const float cosM3 = cos(225 * M_PI / 180);

static const float sinM4 = sin(150 * M_PI / 180);
static const float cosM4 = cos(150 * M_PI / 180);

// 機体速度からオムニ回転数に変換
void setTargetOmniAngle(target_t * target)
{
  float rotation_omega_motor = 0;

  rotation_omega_motor = ROBOT_RADIUS * target->yaw_rps;

  for (int i = 0; i < 4; i++) {
    target->omni_angle[i].pre_tar_rps = target->omni_angle[i].current_tar_rps;
  }

  target->omni_angle[0].current_tar_rps = ((target->local_vel_now[1] * sinM1) + (target->local_vel_now[0] * cosM1) + rotation_omega_motor) / rotation_length_omni;
  target->omni_angle[1].current_tar_rps = ((target->local_vel_now[1] * sinM2) + (target->local_vel_now[0] * cosM2) + rotation_omega_motor) / rotation_length_omni;
  target->omni_angle[2].current_tar_rps = ((target->local_vel_now[1] * sinM3) + (target->local_vel_now[0] * cosM3) + rotation_omega_motor) / rotation_length_omni;
  target->omni_angle[3].current_tar_rps = ((target->local_vel_now[1] * sinM4) + (target->local_vel_now[0] * cosM4) + rotation_omega_motor) / rotation_length_omni;

  for (int i = 0; i < 4; i++) {
    target->omni_angle[i].angle_rad += 2 * M_PI * target->omni_angle[i].current_tar_rps / MAIN_LOOP_CYCLE;
    if (target->omni_angle[i].angle_rad > 2 * M_PI) {
      target->omni_angle[i].angle_rad -= 2 * M_PI;
    } else if (target->omni_angle[i].angle_rad < 0) {
      target->omni_angle[i].angle_rad += 2 * M_PI;
    }

    target->omni_angle[i].tar_aps_acc = target->omni_angle[i].current_tar_rps - target->omni_angle[i].pre_tar_rps;
  }
}

void clearOmniRotationAngleErrorIfStopped(const RobotCommandV2 * ai_cmd, const imu_t * imu, const system_t * sys, target_t * target, const motor_t * motor)
{
  const float yaw_error = getAngleDiff(ai_cmd->target_global_theta, imu->yaw_rad);
  const float yaw_rate = getAngleDiff(imu->yaw_rad, imu->pre_yaw_rad) * MAIN_LOOP_CYCLE;
  /*
   * x方向成分は対向輪の和で相殺される。前側ペアと後側ペアの
   * y方向係数差を使い、4輪誤差から機体回転の共通成分だけを求める。
   */
  const float pair_m1_m4_error =
    0.5f * (getAngleDiff(target->omni_angle[0].angle_rad, motor->angle_rad[0]) + getAngleDiff(target->omni_angle[3].angle_rad, motor->angle_rad[3]));
  const float pair_m2_m3_error =
    0.5f * (getAngleDiff(target->omni_angle[1].angle_rad, motor->angle_rad[1]) + getAngleDiff(target->omni_angle[2].angle_rad, motor->angle_rad[2]));
  const float rotation_error = (sinM1 * pair_m2_m3_error - sinM2 * pair_m1_m4_error) / (sinM1 - sinM2);

  target->omni_rotation_angle_error = rotation_error;
  target->omni_rotation_clear_step = 0.0f;

  bool stable = fabsf(target->local_vel[0]) < ANGLE_CLEAR_LOCAL_VELOCITY_THRESHOLD_MPS &&
                fabsf(target->local_vel[1]) < ANGLE_CLEAR_LOCAL_VELOCITY_THRESHOLD_MPS &&
                fabsf(target->local_vel_now[0]) < ANGLE_CLEAR_LOCAL_VELOCITY_THRESHOLD_MPS &&
                fabsf(target->local_vel_now[1]) < ANGLE_CLEAR_LOCAL_VELOCITY_THRESHOLD_MPS &&
                fabsf(yaw_error) < THETA_CONTROL_DEAD_ZONE_RADIAN && fabsf(yaw_rate) < ANGLE_CLEAR_YAW_RATE_THRESHOLD_RAD_PER_SEC &&
                fabsf(target->yaw_rps) < ANGLE_CLEAR_YAW_RATE_THRESHOLD_RAD_PER_SEC;

  for (int i = 0; i < 4 && stable; i++) {
    stable = fabsf(target->omni_angle[i].current_tar_rps) < ANGLE_CLEAR_WHEEL_RPS_THRESHOLD &&
             (sys->system_time_ms - motor->latest_rx_time_ms[i]) <= ANGLE_CLEAR_CAN_RX_MAX_AGE_MS;
  }

  if (!stable) {
    target->omni_angle_clear_stable_count = 0U;
    target->omni_angle_clear_active = false;
    return;
  }

  if (target->omni_angle_clear_stable_count < ANGLE_CLEAR_STABLE_CYCLES) {
    target->omni_angle_clear_stable_count++;
  }
  if (target->omni_angle_clear_stable_count < ANGLE_CLEAR_STABLE_CYCLES) {
    target->omni_angle_clear_active = false;
    return;
  }

  float clear_step = clampSize(rotation_error * ANGLE_CLEAR_ALPHA, ANGLE_CLEAR_MAX_STEP_RAD);
  if (fabsf(rotation_error) < ANGLE_CLEAR_FINISH_THRESHOLD_RAD) {
    clear_step = rotation_error;
  }

  target->omni_angle_clear_active = true;
  target->omni_rotation_clear_step = clear_step;
  for (int i = 0; i < 4; i++) {
    target->omni_angle[i].angle_rad -= clear_step;
    if (target->omni_angle[i].angle_rad >= 2.0f * M_PI) {
      target->omni_angle[i].angle_rad -= 2.0f * M_PI;
    } else if (target->omni_angle[i].angle_rad < 0.0f) {
      target->omni_angle[i].angle_rad += 2.0f * M_PI;
    }
  }
}

//
void omniAngleControl(target_t * target, output_t * output, motor_t * motor)
{
  for (int i = 0; i < 4; i++) {
    // angle_radは回転方向逆
    target->omni_angle[i].diff = getAngleDiff(target->omni_angle[i].angle_rad, motor->angle_rad[i]);

    target->omni_angle[i].real_rps = motor->rps[i];

    float rps_diff = target->omni_angle[i].real_rps - target->omni_angle[i].current_tar_rps;
    float angle_diff = target->omni_angle[i].diff;
    float gain_cofe = 1.0;
    /*     if (fabs(rps_diff) < 1) {
      rps_diff = 0;
    } else if (rps_diff < -1) {
      angle_diff += 1;
    } else if (rps_diff > 1) {
      angle_diff -= 1;
    } */

    if (target->omni_angle[i].tar_aps_acc * rps_diff < 0) {
      target->omni_angle[i].weak_flag = 0;
      gain_cofe = 1;

    } else {
      target->omni_angle[i].weak_flag = 100;
      gain_cofe = 1;
    }

    // diff 30:0.3, 0.3x50
    // だいたいピークで15ぐらいなのでいい感じっぽい
    target->omni_angle[i].rps_diff = rps_diff;
    target->omni_angle[i].kp_output = clampSize(angle_diff * target->omni_angle_kp * gain_cofe, 15);  //速度次元ではI項
    target->omni_angle[i].kd_output = -rps_diff * target->omni_angle_kd;                              //速度次元ではP項
    target->omni_angle[i].yaw_output = ROBOT_RADIUS * target->yaw_rps_drag;                           //速度次元ではD項
    target->omni_angle[i].ff_output = target->omni_angle[i].current_tar_rps;

    // 各項を保持し、直進加速ログで左右輪の出力差を確認できるようにする。
    output->motor_voltage[i] = target->omni_angle[i].kp_output + target->omni_angle[i].kd_output + target->omni_angle[i].yaw_output + target->omni_angle[i].ff_output;
  }
}
