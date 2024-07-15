/*
 * robot_control.c
 *
 *  Created on: May 19, 2024
 *      Author: hiroyuki
 */
#include "robot_control.h"

#include "robot_packet.h"
#include "util.h"

// 加速度パラメーター
#define ACCEL_LIMIT (6.0)       // m/ss
#define ACCEL_LIMIT_BACK (4.0)  // m/ss

// 減速方向は制動力を増強 1.5～2.0
#define DEC_BOOST_GAIN (2.0)

// 速度制御の位置に対するフィードバックゲイン
// ~ m/s / m : -250 -> 4cm : 1m/s
#define OUTPUT_GAIN_ODOM_DIFF_KP (150)
#define OUTPUT_GAIN_ODOM_DIFF_KD (2)
// 5でもちょっと遅いかも
// 10にすると立ち上がりが遅くなる

// 上記の出力制限
#define OUTPUT_OUTPUTLIMIT_ODOM_DIFF (30)  //

// 0.3はややデカいかも、0.2は割といい感じ
// 比例値であり、最大値
// output = ACCEL_LIMIT x FF_ACC_OUTPUT_KP
//#define FF_ACC_OUTPUT_KP (0.3)
#define FF_ACC_OUTPUT_KP (0.2)

// radに対するゲインなので値がデカい
#define OMEGA_GAIN_KP (160.0)
#define OMEGA_GAIN_KD (4000.0)

// ドライバ側は 50 rps 制限
// omegaぶんは考慮しない
#define OUTPUT_XY_LIMIT (2.0)  //
//#define OUTPUT_XY_LIMIT (40.0)  //

// omegaぶんの制限

void thetaControl(float target_theta, output_t * output, imu_t * imu)
{
  const float OUTPUT_OMEGA_LIMIT = 10.0;  // ~ rad/s
  //#define OUTPUT_OMEGA_LIMIT (20.0)  // ~ rad/s

  // PID
  output->omega = (getAngleDiff(target_theta, imu->yaw_angle_rad) * OMEGA_GAIN_KP) - (getAngleDiff(imu->yaw_angle_rad, imu->pre_yaw_angle_rad) * OMEGA_GAIN_KD);
  output->omega = clampSize(output->omega, OUTPUT_OMEGA_LIMIT);
  //output->omega = 0;
}

void setOutXero(output_t * output)
{
  output->velocity[0] = 0;
  output->velocity[1] = 0;
  output->omega = 0;
}

void localPositionFeedback(integration_control_t * integ, imu_t * imu, target_t * target, RobotCommandV2 * ai_cmd, omni_t * omni, mouse_t * mouse)
{
  static float pre_vision_pos[2] = {0, 0};
  float vision_vel[2] = {0, 0};
  // 加速時はaccelControlと共通で良い
  // 減速時はodomのズレ､マウスの遅延､Visionの遅延があるので､出力トルク(=加速度)に制約かけて､位置に対してPIDやったほうがいいかも

  for (int i = 0; i < 2; i++) {
    integ->position_diff[i] = ai_cmd->mode_args.position.target_global_pos[i] - integ->vision_based_position[i];
    // + mouse->global_vel[i] * 1;
    vision_vel[i] = (ai_cmd->vision_global_pos[i] - pre_vision_pos[i]) * MAIN_LOOP_CYCLE;  // カタついてしゃーない
    pre_vision_pos[i] = ai_cmd->vision_global_pos[i];

    target->global_vel[i] = integ->position_diff[i] * 50;
  }
  //clampScalarSize(target->global_vel, OUTPUT_XY_LIMIT);

  convertGlobalToLocal(target->global_vel, target->local_vel, imu->yaw_angle_rad);
}

void accelControl(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni)
{
  //target->local_vel[0] = target->global_vel[0];
  //target->local_vel[1] = target->global_vel[1];

  // グローバル座標指令

  target->local_vel[0] = (target->global_vel[0]) * cos(imu->yaw_angle_rad) - (target->global_vel[1]) * sin(imu->yaw_angle_rad);
  target->local_vel[1] = (target->global_vel[0]) * sin(imu->yaw_angle_rad) + (target->global_vel[1]) * cos(imu->yaw_angle_rad);

  // XY -> rad/scalarに変換
  // 座標次元でのみフィードバックを行う。速度次元ではフィードバックを行わない。
  // ここでは目標加速度のみ決定するため、内部の目標速度のみを使用する。
  for (int i = 0; i < 2; i++) {
    acc_vel->vel_error_xy[i] = target->local_vel[i] - target->local_vel_now[i];
  }
  acc_vel->vel_error_scalar = pow(pow(acc_vel->vel_error_xy[0], 2) + pow(acc_vel->vel_error_xy[1], 2), 0.5);
  if (acc_vel->vel_error_xy[0] != 0 || acc_vel->vel_error_xy[1] != 0) {
    acc_vel->vel_error_rad = atan2(acc_vel->vel_error_xy[1], acc_vel->vel_error_xy[0]);
  }

  // スカラは使わず、常に最大加速度
  output->accel[0] = cos(acc_vel->vel_error_rad) * ACCEL_LIMIT;
  output->accel[1] = sin(acc_vel->vel_error_rad) * ACCEL_LIMIT;

  // バック方向だけ加速度制限
  if (output->accel[0] < -(ACCEL_LIMIT_BACK)) {
    output->accel[0] = -(ACCEL_LIMIT_BACK);
  }

  for (int i = 0; i < 2; i++) {
    if (target->local_vel_now[i] * output->accel[i] <= 0) {
      output->accel[i] *= DEC_BOOST_GAIN;
    }

    // 目標座標を追い越した場合、加速度を2倍にして現実の位置に追従
    // 現在座標も速度制御されたタイヤで見ているので、あまりｱﾃにならない
    // これ使える気がする
    /*if ((omni->robot_pos_diff[i] > 0 && output->accel[i] > 0) || (omni->robot_pos_diff[i] < 0 && output->accel[i] < 0)) {
      //output->accel[i] *= 1.5;
    }*/
  }
}

void speedControl(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni)
{
  // 目標速度と差が小さい場合は目標速度をそのまま代入する
  // 目標速度が連続的に変化する場合に適切でないかも
  if (acc_vel->vel_error_scalar <= ACCEL_LIMIT / MAIN_LOOP_CYCLE) {
    target->global_vel_now[0] = (target->local_vel[0]) * cos(imu->yaw_angle_rad) - (target->local_vel[1]) * sin(imu->yaw_angle_rad);
    target->global_vel_now[1] = (target->local_vel[0]) * sin(imu->yaw_angle_rad) + (target->local_vel[1]) * cos(imu->yaw_angle_rad);

    output->accel[0] = 0;
    output->accel[1] = 0;
  }

  // ローカル→グローバル座標系
  // ロボットが回転しても、慣性はグローバル座標系に乗るので、加速度はグローバル座標系に変換してから加算
  // accel
  target->global_vel_now[0] += ((output->accel[0]) * cos(imu->yaw_angle_rad) - (output->accel[1]) * sin(imu->yaw_angle_rad)) / MAIN_LOOP_CYCLE;
  target->global_vel_now[1] += ((output->accel[0]) * sin(imu->yaw_angle_rad) + (output->accel[1]) * cos(imu->yaw_angle_rad)) / MAIN_LOOP_CYCLE;

  // 次回の計算のためにローカル座標系での速度も更新
  target->local_vel_now[0] = target->global_vel_now[0] * cos(-imu->yaw_angle_rad) - target->global_vel_now[1] * sin(-imu->yaw_angle_rad);
  target->local_vel_now[1] = target->global_vel_now[0] * sin(-imu->yaw_angle_rad) + target->global_vel_now[1] * cos(-imu->yaw_angle_rad);

  // 目標座標に変換
  target->global_pos[0] += target->global_vel_now[0] / MAIN_LOOP_CYCLE;  // speed to position
  target->global_pos[1] += target->global_vel_now[1] / MAIN_LOOP_CYCLE;  // speed to position

  // ここから位置制御
  for (int i = 0; i < 2; i++) {
    // 目標速度との乖離が大きい時に応答性を稼ぐためのFF項目
    float local_vel_diff_abs = fabs(target->local_vel[i] - target->local_vel_now[i]) / 0.5;  // 0.5がmax
    if (local_vel_diff_abs > 1) {
      local_vel_diff_abs = 1;
    }
    target->local_vel_ff_factor[i] = output->accel[i] * FF_ACC_OUTPUT_KP * local_vel_diff_abs;

    // targetとodomの差分に上限をつける(吹っ飛び対策)
    // 出力が上限に張り付いたら、出力制限でそれ以上の加速度は出しようがないのでそれに合わせる
    // 0.33
    // pos = 1 odom = 0
    // 1 - 0 > 0.33
    // -> pos = 0 + 0.33
    const float odom_diff_max = (float)OUTPUT_OUTPUTLIMIT_ODOM_DIFF / OUTPUT_GAIN_ODOM_DIFF_KP;
    if (target->global_pos[i] - omni->odom[i] > odom_diff_max) {
      target->global_pos[i] = omni->odom[i] + odom_diff_max;
    } else if (target->global_pos[i] - omni->odom[i] < -odom_diff_max) {
      target->global_pos[i] = omni->odom[i] - odom_diff_max;
    }

    // pos = 0,
    omni->global_odom_diff[i] = target->global_pos[i] - omni->odom[i];
  }

  // グローバル→ローカル座標系
  omni->robot_pos_diff[0] = omni->global_odom_diff[0] * cos(-imu->yaw_angle_rad) - omni->global_odom_diff[1] * sin(-imu->yaw_angle_rad);
  omni->robot_pos_diff[1] = omni->global_odom_diff[0] * sin(-imu->yaw_angle_rad) + omni->global_odom_diff[1] * cos(-imu->yaw_angle_rad);

  /*for (int i = 0; i < 2; i++) {
    if ((-omni->robot_pos_diff[i]) * output->accel[i] < 0) {
      omni->robot_pos_diff[i] = 0;
    }
  }*/

  // 加速度と同じぐらいのoutput->velocityを出したい
  output->velocity[0] = omni->robot_pos_diff[0] * OUTPUT_GAIN_ODOM_DIFF_KP + target->local_vel_ff_factor[0] - target->local_vel_now[0] * OUTPUT_GAIN_ODOM_DIFF_KD;
  output->velocity[1] = omni->robot_pos_diff[1] * OUTPUT_GAIN_ODOM_DIFF_KP + target->local_vel_ff_factor[1] - target->local_vel_now[1] * OUTPUT_GAIN_ODOM_DIFF_KD;
}

void robotControl(
  system_t * sys, RobotCommandV2 * ai_cmd, imu_t * imu, accel_vector_t * acc_vel, integration_control_t * integ, target_t * target, omni_t * omni, mouse_t * mouse, debug_t * debug, output_t * output)
{
  // 出力しない
  if (sys->main_mode > MAIN_MODE_CMD_DEBUG_MODE) {
    output->velocity[0] = 0;
    output->velocity[1] = 0;
    output->omega = 0;
    return;
  }

  switch (ai_cmd->control_mode) {
    case LOCAL_CAMERA_MODE:
      // これはいつか実装する
      setOutXero(output);
      return;

    case POSITION_TARGET_MODE:
      localPositionFeedback(integ, imu, target, ai_cmd, omni, mouse);
      //accelControl(acc_vel, output, target, imu, omni);
      //speedControl(acc_vel, output, target, imu, omni);
      output->velocity[0] = target->local_vel[0];
      output->velocity[1] = target->local_vel[1];
      clampScalarSize(output->velocity, OUTPUT_XY_LIMIT);
      thetaControl(ai_cmd->target_global_theta, output, imu);
      break;

    case SIMPLE_VELOCITY_TARGET_MODE:
      target->global_vel[0] = ai_cmd->mode_args.simple_velocity.target_global_vel[0];
      target->global_vel[1] = ai_cmd->mode_args.simple_velocity.target_global_vel[1];
      accelControl(acc_vel, output, target, imu, omni);
      speedControl(acc_vel, output, target, imu, omni);
      clampScalarSize(output->velocity, OUTPUT_XY_LIMIT);
      thetaControl(ai_cmd->target_global_theta, output, imu);
      return;

    case VELOCITY_TARGET_WITH_TRAJECTORY_MODE:
      // これはいつか実装する
      setOutXero(output);
      thetaControl(ai_cmd->target_global_theta, output, imu);  // thetaだけ制御する
      return;

    default:
      setOutXero(output);
      return;
  }
}