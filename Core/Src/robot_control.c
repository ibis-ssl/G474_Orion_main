/*
 * robot_control.c
 *
 *  Created on: May 19, 2024
 *      Author: hiroyuki
 */
#include "robot_control.h"

#include "robot_packet.h"
#include "util.h"

// スカラ速度制限(速度制御モード)
#define SPEED_SCALAR_LIMIT (7.0)

/* 位置フィードバック制御用パラメーター */

// 加速度パラメーター
#define ACCEL_LIMIT (6)       // m/ss
#define ACCEL_LIMIT_BACK (4)  // m/ss

#define ACCEL_TO_OUTPUT_GAIN (0.5)

// 減速方向は制動力を増強 1.5～2.0
#define DEC_BOOST_GAIN (1.0)

/*  速度制御用パラメーター  */

// 減速時の加速度の倍率
#define ACCEL_DECCEL_MULTI (1.8)

// 後方へ加速(減速)するときの倍率
#define ACCEL_BACK_SIDE_MULTI (0.7)

// 速度制御の位置に対するフィードバックゲイン
// ~ m/s / m : -250 -> 4cm : 1m/s
#define OUTPUT_GAIN_KP_ODOM_DIFF (150)
// 上記の出力制限、速度に追従できていれば小さいほうが良い
#define OUTPUT_LIMIT_ODOM_DIFF (2)

// モーター側でパラメーター併せてあるので1.0で良いはず(念の為調整用)
#define OUTPUT_GAIN_TAR_TO_VEL (1)

// 指令速度と内部目標速度の差が大きい時に、加速の立ち上がりを良くするためだけのパラメーター
// 上限がついているので、大きくはならない。
// 加速終了時を滑らかにするために比例項としている。
// 0.3はややデカいかも、0.2は割といい感じ
// output = ACCEL_LIMIT x FF_ACC_OUTPUT_KP
//#define FF_ACC_OUTPUT_KP (0.3)
#define FF_ACC_OUTPUT_KP (0.2)

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

static void setOutZero(output_t * output)
{
  output->velocity[0] = 0;
  output->velocity[1] = 0;
  output->omega = 0;
}

static void localPositionFeedback(integration_control_t * integ, imu_t * imu, target_t * target, RobotCommandV2 * ai_cmd, omni_t * omni, mouse_t * mouse, accel_vector_t * acc_vel, output_t * output)
{
  for (int i = 0; i < 2; i++) {
    // 今はvision_availableがリアルタイムでないので､一旦visionの値をそのまま使う
    //integ->position_diff[i] = ai_cmd->mode_args.position.target_global_pos[i] - integ->vision_based_position[i];
    integ->position_diff[i] = ai_cmd->mode_args.position.target_global_pos[i] - ai_cmd->vision_global_pos[i];

    // + mouse->global_vel[i] * 1;

    target->global_vel[i] = integ->position_diff[i] * 40;
    // 計算上の都合で､速度の向きをthetaではなくxyで与える｡そのため値なのでゲインの意味は少ない
  }
  clampScalarSize(target->global_vel, ai_cmd->linear_velocity_limit);  // ここの速度指定はほぼ意味ない

  //デバッグ用の一時実装
  target->target_pos_dist_scalar = calcScalar(integ->position_diff[0], integ->position_diff[1]);

  //convertGlobalToLocal(integ->position_diff, integ->local_target_diff, imu->yaw_angle_rad);
  // つかってない

  float using_omni_speed[2];
  using_omni_speed[0] = omni->local_odom_speed_mvf[0];
  using_omni_speed[1] = omni->local_odom_speed_mvf[1];

  // 0^2 + v^2 = 2ax
  // v = √2ax
  // 機体の向きによって制動力が変化するのを考慮したほうが良い｡
  // aは減速なので DEC_BOOST_GAIN 倍する
  // 指令値とイコールだと制御余裕がない
  target->target_scalar_vel = pow(2 * (ACCEL_LIMIT * DEC_BOOST_GAIN * 0.75) * target->target_pos_dist_scalar, 0.5) + ai_cmd->mode_args.position.terminal_velocity;

  // v^2 = 2 * acc * 2 * 0.1
  // デバッグ用の1m/s｡本当はai_cmdの値を使う
  float linear_velocity_limit = ai_cmd->linear_velocity_limit;
  //float linear_velocity_limit = 1.5;
  if (target->target_scalar_vel > linear_velocity_limit) {
    target->to_stop_mode_flag = false;
  } else {
    target->to_stop_mode_flag = true;
  }
  target->target_scalar_vel = clampSize(target->target_scalar_vel, linear_velocity_limit);
  // 目標地点付近での制御は別で用意していいかも
  if (target->target_pos_dist_scalar < 0.03) {
    target->target_scalar_vel = 0;
  }

  target->target_vel_angle = atan2(target->global_vel[1], target->global_vel[0]);
  //float dummy_speed[2] = {1, 0};
  //convertLocalToGlobal(dummy_speed, target->global_odom_speed, imu->yaw_angle_rad);
  convertLocalToGlobal(using_omni_speed, target->global_odom_speed, imu->yaw_angle_rad);

  // 関数名と違うけどターゲット速度座標系に変換
  convertGlobalToLocal(target->global_odom_speed, target->current_speed_crd, target->target_vel_angle);
  target->target_crd_acc[0] = (target->target_scalar_vel - target->current_speed_crd[0]) * 7.5;  //程々に下げたほうがいいがち
  target->target_crd_acc[1] = -target->current_speed_crd[1] * 5;                                 //20だと発振

  if (target->target_crd_acc[0] > ACCEL_LIMIT) {
    target->target_crd_acc[0] = ACCEL_LIMIT;
  } else if (target->target_crd_acc[0] < -ACCEL_LIMIT * DEC_BOOST_GAIN) {
    target->target_crd_acc[0] = -ACCEL_LIMIT * DEC_BOOST_GAIN;  // 減速方向のみ2倍
  }
  if (!target->to_stop_mode_flag && target->target_crd_acc[0] < 0) {  //速度制限への追従は控えめ
    target->target_crd_acc[0] /= 10;
  }
  target->target_crd_acc[1] = clampSize(target->target_crd_acc[1], ACCEL_LIMIT * DEC_BOOST_GAIN);  // 常に減速のため､2倍

  convertLocalToGlobal(target->target_crd_acc, target->global_acc, target->target_vel_angle);
  convertGlobalToLocal(target->global_acc, output->accel, imu->yaw_angle_rad);

  // バック方向だけ加速度制限
  if (output->accel[0] < -(ACCEL_LIMIT_BACK)) {
    output->accel[0] = -(ACCEL_LIMIT_BACK);
  }

  if (output->accel[0] > 0) {
    // 精度悪いのでまだ使えない
    //using_omni_speed[0] = omni->local_odom_speed_mvf[2];
  }

  //output->velocity[0] = using_omni_speed[0] + output->accel[0] * ACCEL_TO_OUTPUT_GAIN;
  output->velocity[0] = using_omni_speed[0] + output->accel[0] * 0.4;
  output->velocity[1] = using_omni_speed[1] + output->accel[1] * 0.7;  //左右が動き悪いので出力段で増やす

  /*if (target->target_scalar_vel == 0 && target->current_speed_crd[0] < 0.1 && target->current_speed_crd[0] < 0.1) {
    output->velocity[0] = 0;
    output->velocity[1] = 0;
  }*/
}

inline static void setLocalTargetSpeed(RobotCommandV2 * ai_cmd, target_t * target, imu_t * imu)
{
  target->global_vel[0] = ai_cmd->mode_args.simple_velocity.target_global_vel[0];
  target->global_vel[1] = ai_cmd->mode_args.simple_velocity.target_global_vel[1];
  clampScalarSize(target->global_vel, SPEED_SCALAR_LIMIT);

  convertGlobalToLocal(target->global_vel, target->local_vel, imu->yaw_angle_rad);
}

static void setTargetAccel(RobotCommandV2 * ai_cmd, accel_vector_t * acc_vel)
{
  //acc_vel->accel_target = 7.0;
  //return;
  if (ai_cmd->acceleration_limit < 3.0 || ai_cmd->acceleration_limit > 20) {
    acc_vel->accel_target = 5.0;
  } else {
    acc_vel->accel_target = ai_cmd->acceleration_limit;
  }
}

static void accelControl(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni)
{
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
  output->accel[0] = cos(acc_vel->vel_error_rad) * acc_vel->accel_target;
  output->accel[1] = sin(acc_vel->vel_error_rad) * acc_vel->accel_target;

  // バック方向だけ加速度制限
  if (output->accel[0] < -(acc_vel->accel_target * ACCEL_BACK_SIDE_MULTI)) {
    output->accel[0] = -(acc_vel->accel_target * ACCEL_BACK_SIDE_MULTI);
  }

  // 減速時に1.8倍
  // いずれ指令値側で増やすので、消す
  for (int i = 0; i < 2; i++) {
    if (target->local_vel_now[i] * output->accel[i] <= 0) {
      output->accel[i] *= ACCEL_DECCEL_MULTI;
    }

    // 目標座標を追い越した場合、加速度を2倍にして現実の位置に追従
    // 現在座標も速度制御されたタイヤで見ているので、あまりｱﾃにならない
    // これ使える気がする
    /*if ((omni->robot_pos_diff[i] > 0 && output->accel[i] > 0) || (omni->robot_pos_diff[i] < 0 && output->accel[i] < 0)) {
      //output->accel[i] *= 1.5;
    }*/
  }
}

static void speedControl(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni, RobotCommandV2 * ai_cmd)
{
  // 目標速度と差が小さい場合は目標速度をそのまま代入する
  // 目標速度が連続的に変化する場合に適切でないかも
  if (acc_vel->vel_error_scalar <= ACCEL_LIMIT / MAIN_LOOP_CYCLE && ai_cmd->control_mode == SIMPLE_VELOCITY_TARGET_MODE) {
    //convertLocalToGlobal(target->local_vel, target->global_vel_now, imu->yaw_angle_rad);
    target->global_vel_now[0] = (target->local_vel[0]) * cos(imu->yaw_angle_rad) - (target->local_vel[1]) * sin(imu->yaw_angle_rad);
    target->global_vel_now[1] = (target->local_vel[0]) * sin(imu->yaw_angle_rad) + (target->local_vel[1]) * cos(imu->yaw_angle_rad);

    output->accel[0] = 0;
    output->accel[1] = 0;
  }

  // ローカル→グローバル座標系
  // ロボットが回転しても、慣性はグローバル座標系に乗るので、加速度はグローバル座標系に変換してから加算
  // accel
  static float global_vel_diff[2] = {0};
  convertLocalToGlobal(output->accel, global_vel_diff, imu->yaw_angle_rad);
  target->global_vel_now[0] += global_vel_diff[0] / MAIN_LOOP_CYCLE;
  target->global_vel_now[1] += global_vel_diff[1] / MAIN_LOOP_CYCLE;

  // 次回の計算のためにローカル座標系での速度も更新
  convertGlobalToLocal(target->global_vel_now, target->local_vel_now, imu->yaw_angle_rad);

  // 目標座標に変換
  target->global_pos[0] += target->global_vel_now[0] / MAIN_LOOP_CYCLE;  // speed to position
  target->global_pos[1] += target->global_vel_now[1] / MAIN_LOOP_CYCLE;  // speed to position

  // ここから位置制御
  for (int i = 0; i < 2; i++) {
    // 目標速度との乖離が大きい時に応答性を稼ぐためのFF項目
    float local_vel_diff_abs = fabs(target->local_vel[i] - target->local_vel_now[i]) / 0.5;  // 速度差 0.5 [m/s] で最大
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
    const float odom_diff_max = (float)OUTPUT_LIMIT_ODOM_DIFF / (float)OUTPUT_GAIN_KP_ODOM_DIFF;  //0.2
    float odom_diff = target->global_pos[i] - omni->odom[i];
    if (odom_diff > odom_diff_max) {
      target->global_pos[i] = omni->odom[i] + odom_diff_max;
    } else if (odom_diff < -odom_diff_max) {
      target->global_pos[i] = omni->odom[i] - odom_diff_max;
    }

    // pos = 0,
    omni->global_odom_diff[i] = odom_diff;
  }

  // グローバル→ローカル座標系
  convertGlobalToLocal(omni->global_odom_diff, omni->robot_pos_diff, imu->yaw_angle_rad);

  // 加速方向を切り替えた時、位置フィードバック項目の遅れが出るので、差分を打ち消す必要がある。未確認
  /*for (int i = 0; i < 2; i++) {
    if ((-omni->robot_pos_diff[i]) * output->accel[i] < 0) {
      omni->robot_pos_diff[i] = 0;
    }
  }*/

  // 加速度と同じぐらいのoutput->velocityを出したい
  output->velocity[0] = omni->robot_pos_diff[0] * OUTPUT_GAIN_KP_ODOM_DIFF + target->local_vel_ff_factor[0] + target->local_vel_now[0] * OUTPUT_GAIN_TAR_TO_VEL;
  output->velocity[1] = omni->robot_pos_diff[1] * OUTPUT_GAIN_KP_ODOM_DIFF + target->local_vel_ff_factor[1] + target->local_vel_now[1] * OUTPUT_GAIN_TAR_TO_VEL;
}

static void simpleSpeedControl(output_t * output, target_t * target, imu_t * imu, omni_t * omni)
{
  target->local_vel[0] = (target->global_vel[0]) * cos(imu->yaw_angle_rad) - (target->global_vel[1]) * sin(imu->yaw_angle_rad);
  target->local_vel[1] = (target->global_vel[0]) * sin(imu->yaw_angle_rad) + (target->global_vel[1]) * cos(imu->yaw_angle_rad);

  //convertGlobalToLocal(target->local_vel, target->global_vel, imu->yaw_angle_rad);

  output->accel[0] = (target->local_vel[0] - omni->local_odom_speed[0]) * 5;
  if (output->accel[0] > ACCEL_LIMIT * 2) {
    output->accel[0] = ACCEL_LIMIT * 2;
  } else if (output->accel[0] < -ACCEL_LIMIT * 1.5) {
    output->accel[0] = -ACCEL_LIMIT * 1.5;
  }

  output->accel[1] = (target->local_vel[1] - omni->local_odom_speed[1]) * 5;
  output->accel[1] = clampSize(output->accel[1], ACCEL_LIMIT * 1.5);

  clampScalarSize(output->accel, ACCEL_LIMIT * 3);

  // 原則強化
  for (int i = 0; i < 2; i++) {
    if (fabs(target->local_vel[i]) < fabs(omni->local_odom_speed_mvf[i])) {
      output->accel[i] *= 1.5;
    }
  }
}

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
        simpleAccelToOutput(omni, output);
      } else {
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
        simpleAccelToOutput(omni, output);
      }
      //simpleSpeedControl(output, target, imu, omni);
      //accelControl(acc_vel, output, target, imu, omni);
      //speedControl(acc_vel, output, target, imu, omni);
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