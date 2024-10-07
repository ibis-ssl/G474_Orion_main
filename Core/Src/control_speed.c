
#include "control_speed.h"

#include "util.h"

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

void setTargetAccel(system_t * sys, RobotCommandV2 * ai_cmd, accel_vector_t * acc_vel)
{
  if (sys->main_mode == MAIN_MODE_MANUAL_CONTROL) {
    acc_vel->accel_target = 7.0;
    return;
  }

  if (ai_cmd->acceleration_limit < 3.0 || ai_cmd->acceleration_limit > 20) {
    acc_vel->accel_target = 5.0;
  } else {
    acc_vel->accel_target = ai_cmd->acceleration_limit;
  }
}

void accelControl(system_t * sys, accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni)
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
  if (sys->main_mode == MAIN_MODE_MANUAL_CONTROL) {
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
}

void clearPosDiffSpeedControl(target_t * target, omni_t * omni)
{
  for (int i = 0; i < 2; i++) {
    target->global_pos[i] = omni->odom[i];
  }
}

void speedControl(accel_vector_t * acc_vel, output_t * output, target_t * target, imu_t * imu, omni_t * omni, RobotCommandV2 * ai_cmd)
{
  // 目標速度と差が小さい場合は目標速度をそのまま代入する
  // 目標速度が連続的に変化する場合に適切でないかも
  if (acc_vel->vel_error_scalar <= acc_vel->accel_target / MAIN_LOOP_CYCLE && ai_cmd->control_mode == SIMPLE_VELOCITY_TARGET_MODE) {
    //convertLocalToGlobal(target->local_vel, target->global_vel_now, imu->yaw_rad);
    target->global_vel_now[0] = (target->local_vel[0]) * cos(imu->yaw_rad) - (target->local_vel[1]) * sin(imu->yaw_rad);
    target->global_vel_now[1] = (target->local_vel[0]) * sin(imu->yaw_rad) + (target->local_vel[1]) * cos(imu->yaw_rad);

    output->accel[0] = 0;
    output->accel[1] = 0;
  }

  // ローカル→グローバル座標系
  // ロボットが回転しても、慣性はグローバル座標系に乗るので、加速度はグローバル座標系に変換してから加算
  // accel
  static float global_vel_diff[2] = {0};
  convertLocalToGlobal(output->accel, global_vel_diff, imu->yaw_rad);
  target->global_vel_now[0] += global_vel_diff[0] / MAIN_LOOP_CYCLE;
  target->global_vel_now[1] += global_vel_diff[1] / MAIN_LOOP_CYCLE;

  // 次回の計算のためにローカル座標系での速度も更新
  convertGlobalToLocal(target->global_vel_now, target->local_vel_now, imu->yaw_rad);

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
  convertGlobalToLocal(omni->global_odom_diff, omni->robot_pos_diff, imu->yaw_rad);

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
