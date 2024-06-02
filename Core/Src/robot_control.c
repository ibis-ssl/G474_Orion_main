/*
 * robot_control.c
 *
 *  Created on: May 19, 2024
 *      Author: hiroyuki
 */

#include "robot_control.h"

#include "util.h"

// 加速度パラメーター
#define ACCEL_LIMIT (4.0)       // m/ss
#define ACCEL_LIMIT_BACK (4.0)  // m/ss

// 速度制御の位置に対するフィードバックゲイン
// ~ m/s / m : -250 -> 4cm : 1m/s
#define OUTPUT_GAIN_ODOM_DIFF_KP (150)
// 上記の出力制限
#define OUTPUT_OUTPUT_LIMIT_ODOM_DIFF (50)  //

// 0.3はややデカすぎ、0.2は割といい感じ
// accel x KP
//#define FF_ACC_OUTPUT_KP (0.3)
#define FF_ACC_OUTPUT_KP (0.2)

// radに対するゲインなので値がデカい
#define OMEGA_GAIN_KP (160.0)
#define OMEGA_GAIN_KD (4000.0)

// ドライバ側は 50 rps 制限
// omegaぶんは考慮しない
#define OUTPUT_XY_LIMIT (60.0)  //
// omegaぶんの制限
#define OUTPUT_OMEGA_LIMIT (20.0)  // ~ rad/s

void theta_control(float target_theta, accel_vector_t * acc_vel, output_t * output, imu_t * imu)
{
  // PID
  output->omega = (getAngleDiff(target_theta, imu->yaw_angle_rad) * OMEGA_GAIN_KP) - (getAngleDiff(imu->yaw_angle_rad, imu->pre_yaw_angle_rad) * OMEGA_GAIN_KD);

  if (output->omega > OUTPUT_OMEGA_LIMIT) {
    output->omega = OUTPUT_OMEGA_LIMIT;
  }
  if (output->omega < -OUTPUT_OMEGA_LIMIT) {
    output->omega = -OUTPUT_OMEGA_LIMIT;
  }
  //output->omega = 0;
}

void local_feedback(integration_control_t * integ, imu_t * imu, system_t * sys, target_t * target, ai_cmd_t * ai_cmd, omni_t * omni, mouse_t * mouse)
{
  const float CMB_CTRL_FACTOR_LIMIT = (2.0);     // [m/s]
  const float CMB_CTRL_DIFF_DEAD_ZONE = (0.03);  // [m]
  const float CMB_CTRL_GAIN = (10.0);
  const float CMB_CTRL_DIFF_LIMIT = (CMB_CTRL_FACTOR_LIMIT / CMB_CTRL_GAIN);
  bool in_dead_zone_flag = false;

  // デバッグモードでは勝手に諸々書き換える
  if (sys->main_mode == MAIN_MODE_COMBINATION_CONTROL) {
  } else {
    for (int i = 0; i < 2; i++) {
      integ->vision_based_position[i] = -mouse->odom[i];
      integ->position_diff[i] = ai_cmd->local_target_speed[i] / 10 - integ->vision_based_position[i];
    }
  }

  // グローバル→ローカル座標系
  integ->local_target_diff[0] = integ->position_diff[0] * cos(-imu->yaw_angle_rad) - integ->position_diff[1] * sin(-imu->yaw_angle_rad);
  integ->local_target_diff[1] = integ->position_diff[0] * sin(-imu->yaw_angle_rad) + integ->position_diff[1] * cos(-imu->yaw_angle_rad);

  for (int i = 0; i < 2; i++) {
    // デバッグ用にomni->odomをそのままと、target_posにspeed使う
    // 速度制御はodomベースなのでちょっとおかしなことになる
    //integ->local_target_diff[i] = omni->odom[i] - ai_cmd->local_target_speed[i];

    // 精密性はそれほどいらないので、振動対策に不感帯入れる
    // ただの不感帯よりも、スレッショルドとか入れたほうが良いかも
    if (integ->local_target_diff[i] < CMB_CTRL_DIFF_DEAD_ZONE && integ->local_target_diff[i] > -CMB_CTRL_DIFF_DEAD_ZONE) {
      in_dead_zone_flag = true;
    } else {
      in_dead_zone_flag = false;
      // XYで再利用しているのでfalseにもする
    }

    // ゲインは x10
    // 吹き飛び対策で+-3.0 m/sを上限にする
    /*if (integ->local_target_diff[i] < -CMB_CTRL_DIFF_LIMIT) {
      integ->local_target_diff[i] = -CMB_CTRL_DIFF_LIMIT;
    } else if (integ->local_target_diff[i] > CMB_CTRL_DIFF_LIMIT) {
      integ->local_target_diff[i] = CMB_CTRL_DIFF_LIMIT;
    }*/

    if (sys->main_mode == MAIN_MODE_COMBINATION_CONTROL) {
      // 位置フィードバック項目のx10はゲイン (ベタ打ち)
      /*if (ai_cmd->local_target_speed[i] * integ->local_target_diff[i] < 0) {                                  // 位置フィードバック項が制動方向の場合
        target->velocity[i] = ai_cmd->local_target_speed[i] + (integ->local_target_diff[i] * CMB_CTRL_GAIN);  //ローカル統合制御あり
      } else {
        target->velocity[i] = ai_cmd->local_target_speed[i];  // ローカル統合制御なし
      }*/
      //target->velocity[i] = (integ->local_target_diff[i] * CMB_CTRL_GAIN);  //ローカル統合制御あり
      if (fabs(2 * ACCEL_LIMIT_BACK * 2 * 1.0 * integ->local_target_diff[i]) < target->local_vel_now[i] * target->local_vel_now[i] || in_dead_zone_flag) {
        target->velocity[i] = 0;
        //
      } else {
        target->velocity[i] = integ->local_target_diff[i] * CMB_CTRL_GAIN;

        // 指令値を速度制限として適用
        if (target->velocity[i] > CMB_CTRL_FACTOR_LIMIT) {
          target->velocity[i] = CMB_CTRL_FACTOR_LIMIT;
        } else if (target->velocity[i] < -CMB_CTRL_FACTOR_LIMIT) {
          target->velocity[i] = -CMB_CTRL_FACTOR_LIMIT;
        }
      }
    } else {
      // 2 x acc x X = V^2
      // acc : ACCEL_LIMIT_BACK * 2
      // ピッタリだとたまにオーバーシュートしてしまうので0.8かける
      // 動いてるけど不感帯と相性悪いっぽい。スレッショルドかなにか必要かも
      if (fabs(2 * ACCEL_LIMIT_BACK * 2 * 0.5 * integ->local_target_diff[i]) < target->local_vel_now[i] * target->local_vel_now[i] || in_dead_zone_flag) {
        /*if (integ->local_target_diff[i] > 0) {
          target->velocity[i] = pow(fabs(2 * ACCEL_LIMIT_BACK * 2 * 0.5 * integ->local_target_diff[i]), 0.5);
        } else {
          target->velocity[i] = -pow(fabs(2 * ACCEL_LIMIT_BACK * 2 * 0.5 * integ->local_target_diff[i]), 0.5);
        }*/
        target->velocity[i] = 0;
        //
      } else {
        target->velocity[i] = integ->local_target_diff[i] * CMB_CTRL_GAIN - target->local_vel_now[i] * 3;

        // 指令値を速度制限として適用
        if (target->velocity[i] > CMB_CTRL_FACTOR_LIMIT) {
          target->velocity[i] = CMB_CTRL_FACTOR_LIMIT;
        } else if (target->velocity[i] < -CMB_CTRL_FACTOR_LIMIT) {
          target->velocity[i] = -CMB_CTRL_FACTOR_LIMIT;
        }
      }
      //target->velocity[i] = ai_cmd->local_target_speed[i];
    }
  }
}

void accel_control(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni)
{
  target->local_vel[0] = target->velocity[0];
  target->local_vel[1] = target->velocity[1];

  // グローバル座標指令
  //target->local_vel[0] = (target->velocity[0]) * cos(imu->yaw_angle_rad) - (target->velocity[1]) * sin(imu->yaw_angle_rad);
  //target->local_vel[1] = (target->velocity[0]) * sin(imu->yaw_angle_rad) + (target->velocity[1]) * cos(imu->yaw_angle_rad);

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

  // 減速方向は制動力2倍
  // 2倍は流石に無理があるので1.8 → やっぱ2.0
  for (int i = 0; i < 2; i++) {
    if (target->local_vel_now[i] * output->accel[i] <= 0) {
      output->accel[i] *= 1.5;
    }

    // 目標座標を追い越した場合、加速度を2倍にして現実の位置に追従
    // 現在座標も速度制御されたタイヤで見ているので、あまりｱﾃにならない
    // これ使える気がする
    /*if ((omni->robot_pos_diff[i] > 0 && output->accel[i] > 0) || (omni->robot_pos_diff[i] < 0 && output->accel[i] < 0)) {
      //output->accel[i] *= 1.5;
    }*/
  }
}

void speed_control(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni)
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
    // 応答性を稼ぐためのFF項目
    target->local_vel_ff_factor[i] = output->accel[i] * FF_ACC_OUTPUT_KP;

    // targetとodomの差分に上限をつける(吹っ飛び対策)
    // 出力が上限に張り付いたら、出力制限でそれ以上の加速度は出しようがないのでそれに合わせる
    const float odom_diff_max = (float)OUTPUT_OUTPUT_LIMIT_ODOM_DIFF / OUTPUT_GAIN_ODOM_DIFF_KP;
    // 0.33
    // pos = 1 odom = 0
    // 1 - 0 > 0.33
    // -> pos = 0 + 0.33
    /*if (target->global_pos[i] - omni->odom[i] > odom_diff_max) {
      target->global_pos[i] = omni->odom[i] + odom_diff_max;
    } else if (target->global_pos[i] - omni->odom[i] < -odom_diff_max) {
      target->global_pos[i] = omni->odom[i] - odom_diff_max;
    }*/

    // pos = 0,
    omni->global_odom_diff[i] = omni->odom[i] - target->global_pos[i];
  }

  // グローバル→ローカル座標系
  omni->robot_pos_diff[0] = omni->global_odom_diff[0] * cos(-imu->yaw_angle_rad) - omni->global_odom_diff[1] * sin(-imu->yaw_angle_rad);
  omni->robot_pos_diff[1] = omni->global_odom_diff[0] * sin(-imu->yaw_angle_rad) + omni->global_odom_diff[1] * cos(-imu->yaw_angle_rad);

  // 加速度と同じぐらいのoutput->velocityを出したい
  output->velocity[0] = -omni->robot_pos_diff[0] * OUTPUT_GAIN_ODOM_DIFF_KP + target->local_vel_ff_factor[0];
  output->velocity[1] = -omni->robot_pos_diff[1] * OUTPUT_GAIN_ODOM_DIFF_KP + target->local_vel_ff_factor[1];
}

void output_limit(output_t * output, debug_t * debug)
{
  if (debug->acc_step_down_flag) {
    debug->limited_output = 0;  //スリップしてたら移動出力を0にする(仮)
  } else {
    debug->limited_output = OUTPUT_XY_LIMIT;
  }

  float limit_gain = 0;
  if (output->velocity[0] > debug->limited_output) {
    limit_gain = output->velocity[0] / debug->limited_output;
    output->velocity[0] = debug->limited_output;
    output->velocity[1] /= limit_gain;
  } else if (output->velocity[0] < -debug->limited_output) {
    limit_gain = -output->velocity[0] / debug->limited_output;
    output->velocity[0] = -debug->limited_output;
    output->velocity[1] /= limit_gain;
  }

  if (output->velocity[1] > debug->limited_output) {
    limit_gain = output->velocity[1] / debug->limited_output;
    output->velocity[1] = debug->limited_output;
    output->velocity[0] /= limit_gain;
  } else if (output->velocity[1] < -debug->limited_output) {
    limit_gain = -output->velocity[1] / debug->limited_output;
    output->velocity[1] = -debug->limited_output;
    output->velocity[0] /= limit_gain;
  }
}
