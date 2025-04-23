#include "control_omni_angle.h"

#include "math.h"
#include "util.h"

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
    output->motor_voltage[i] = clampSize(angle_diff * target->omni_angle_kp * gain_cofe, 15);  //速度次元ではI項

    // 通常D､微分先行はFFと打ち消し合うのでNG
    output->motor_voltage[i] -= rps_diff * target->omni_angle_kd;  //速度次元ではP項

    output->motor_voltage[i] += ROBOT_RADIUS * target->yaw_rps_drag;    //速度次元ではD項､機体の回転方向の慣性を打ち消す
    output->motor_voltage[i] += target->omni_angle[i].current_tar_rps;  // FF項目
  }
}