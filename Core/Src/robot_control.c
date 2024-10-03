/*
 * robot_control.c
 *
 *  Created on: May 19, 2024
 *      Author: hiroyuki
 */
#include "robot_control.h"

#include "control_pos.h"
#include "control_speed.h"
#include "robot_packet.h"
#include "util.h"

// スカラ速度制限(速度制御モード)
#define SPEED_SCALAR_LIMIT (7.0)

//#define OMEGA_GAIN_KP (160.0)
#define OMEGA_GAIN_KP (30.0)
#define OMEGA_GAIN_KD (4.0 * MAIN_LOOP_CYCLE)

// ドライバ側は 50 rps 制限
// omegaぶんは考慮しない
#define OUTPUT_SCALAR_LIMIT (8.0)  //
//#define OUTPUT_SCALAR_LIMIT (40.0)  //

// omegaぶんの制限

static void thetaControl(RobotCommandV2 * ai_cmd, output_t * output, imu_t * imu, omega_target_t * omega_target)
{
  const float OUTPUT_OMEGA_LIMIT = 20.0;  // ~ rad/s

  float target_diff_angle = getAngleDiff(ai_cmd->target_global_theta, omega_target->current_target);
  target_diff_angle = clampSize(target_diff_angle, ai_cmd->angular_velocity_limit / MAIN_LOOP_CYCLE);
  omega_target->current_target += target_diff_angle;
  // PID
  output->omega = (getAngleDiff(omega_target->current_target, imu->yaw_angle_rad) * OMEGA_GAIN_KP) - (getAngleDiff(imu->yaw_angle_rad, imu->pre_yaw_angle_rad) * OMEGA_GAIN_KD);
  output->omega = clampSize(output->omega, OUTPUT_OMEGA_LIMIT);
  //output->omega = 0;
}

inline void setLocalTargetSpeed(RobotCommandV2 * ai_cmd, target_t * target, imu_t * imu)
{
  target->global_vel[0] = ai_cmd->mode_args.simple_velocity.target_global_vel[0];
  target->global_vel[1] = ai_cmd->mode_args.simple_velocity.target_global_vel[1];
  clampScalarSize(target->global_vel, SPEED_SCALAR_LIMIT);

  convertGlobalToLocal(target->global_vel, target->local_vel, imu->yaw_angle_rad);
}

static void setOutZero(output_t * output)
{
  output->velocity[0] = 0;
  output->velocity[1] = 0;
  output->omega = 0;
}

/*void simpleSpeedControl(output_t * output, target_t * target, imu_t * imu, omni_t * omni)
{
  const float SIMPLE_SPEED_CONTROL_ACCEL = 5.0;
  target->local_vel[0] = (target->global_vel[0]) * cos(imu->yaw_angle_rad) - (target->global_vel[1]) * sin(imu->yaw_angle_rad);
  target->local_vel[1] = (target->global_vel[0]) * sin(imu->yaw_angle_rad) + (target->global_vel[1]) * cos(imu->yaw_angle_rad);

  //convertGlobalToLocal(target->local_vel, target->global_vel, imu->yaw_angle_rad);

  output->accel[0] = (target->local_vel[0] - omni->local_odom_speed[0]) * 5;
  if (output->accel[0] > SIMPLE_SPEED_CONTROL_ACCEL * 2) {
    output->accel[0] = SIMPLE_SPEED_CONTROL_ACCEL * 2;
  } else if (output->accel[0] < -SIMPLE_SPEED_CONTROL_ACCEL * 1.5) {
    output->accel[0] = -SIMPLE_SPEED_CONTROL_ACCEL * 1.5;
  }

  output->accel[1] = (target->local_vel[1] - omni->local_odom_speed[1]) * 5;
  output->accel[1] = clampSize(output->accel[1], SIMPLE_SPEED_CONTROL_ACCEL * 1.5);

  clampScalarSize(output->accel, SIMPLE_SPEED_CONTROL_ACCEL * 3);

  // 原則強化
  for (int i = 0; i < 2; i++) {
    if (fabs(target->local_vel[i]) < fabs(omni->local_odom_speed_mvf[i])) {
      output->accel[i] *= 1.5;
    }
  }
}*/

static void simpleAccelToOutput(omni_t * omni, output_t * output)
{
  output->velocity[0] = omni->local_odom_speed[0] + output->accel[0] * 0.3;
  output->velocity[1] = omni->local_odom_speed[1] + output->accel[1] * 0.6;  //左右が動き悪いので出力段で増やす
}

inline static void clearSpeedContrlValue(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni, RobotCommandV2 * ai_cmd)
{
  acc_vel->vel_error_rad = 0;
  acc_vel->vel_error_scalar = 0;
  for (int i = 0; i < 2; i++) {
    acc_vel->vel_error_xy[i] = 0;  //毎回更新するので多分いらない
    //target->local_vel_now[i] = omni->local_odom_speed_mvf[i];
    target->global_pos[i] = omni->odom[i];
  }
  convertLocalToGlobal(omni->local_odom_speed_mvf, target->global_vel_now, imu->yaw_angle_rad);
}

void robotControl(
  system_t * sys, RobotCommandV2 * ai_cmd, imu_t * imu, accel_vector_t * acc_vel, integration_control_t * integ, target_t * target, omni_t * omni, mouse_t * mouse, debug_t * debug, output_t * output,
  omega_target_t * omega_target)
{
  // 出力しない
  if (sys->main_mode > MAIN_MODE_CMD_DEBUG_MODE) {
    output->velocity[0] = 0;
    output->velocity[1] = 0;
    output->omega = 0;
    return;
  }

  if (sys->stop_flag) {
    clearSpeedContrlValue(acc_vel, output, target, imu, omni, ai_cmd);
  }

  switch (ai_cmd->control_mode) {
    case LOCAL_CAMERA_MODE:
      // これはいつか実装する
      setOutZero(output);
      thetaControl(ai_cmd, output, imu, omega_target);  // thetaだけ制御する
      return;

    case POSITION_TARGET_MODE:
      localPositionFeedback(integ, imu, target, ai_cmd, omni, mouse, acc_vel, output);
      if (sys->main_mode == MAIN_MODE_COMBINATION_CONTROL) {  // 0
        // 速度誤差フィードバックなし
        simpleAccelToOutput(omni, output);
      } else {
        // 速度誤差フィードバックあり
        speedControl(acc_vel, output, target, imu, omni, ai_cmd);
      }
      clampScalarSize(output->velocity, OUTPUT_SCALAR_LIMIT);
      thetaControl(ai_cmd, output, imu, omega_target);
      break;

    case SIMPLE_VELOCITY_TARGET_MODE:
      setLocalTargetSpeed(ai_cmd, target, imu);
      setTargetAccel(ai_cmd, acc_vel);
      if (sys->main_mode == MAIN_MODE_COMBINATION_CONTROL) {  // 0
        accelControl(acc_vel, output, target, imu, omni);
        speedControl(acc_vel, output, target, imu, omni, ai_cmd);
      } else {
        // 位置フィードバックの速度指令部分だけデバッグ用
        accelControl(acc_vel, output, target, imu, omni);
        simpleAccelToOutput(omni, output);
      }
      clampScalarSize(output->velocity, OUTPUT_SCALAR_LIMIT);
      thetaControl(ai_cmd, output, imu, omega_target);
      return;

    case VELOCITY_TARGET_WITH_TRAJECTORY_MODE:
      // これはいつか実装する
      setOutZero(output);
      thetaControl(ai_cmd, output, imu, omega_target);  // thetaだけ制御する
      return;

    default:
      setOutZero(output);
      return;
  }
}