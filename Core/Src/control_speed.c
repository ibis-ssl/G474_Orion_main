
#include "control_speed.h"

#include "util.h"

/*  速度制御用パラメーター  */

// 減速時の加速度の倍率
#define ACCEL_DECCEL_RATIO (1.8)

// 後方へ加速(減速)するときの倍率
#define ACCEL_BACK_SIDE_RATIO (0.7)

void setTargetAccel(RobotCommandV2 * ai_cmd, accel_vector_t * acc_vel)
{
  if (ai_cmd->acceleration_limit < 3.0 || ai_cmd->acceleration_limit > 20) {
    acc_vel->accel_target = 3.0;
  } else {
    acc_vel->accel_target = ai_cmd->acceleration_limit;
  }
}

void accelControl(accel_vector_t * acc_vel, target_t * target)
{
  // XY -> rad/scalarに変換
  // 座標次元でのみフィードバックを行う。速度次元ではフィードバックを行わない。
  // ここでは目標加速度のみ決定するため、内部の目標速度のみを使用する。
  for (int i = 0; i < 2; i++) {
    acc_vel->vel_error_xy[i] = target->local_vel[i] - target->local_vel_now[i];
  }
  acc_vel->vel_error_scalar = sqrtf(powf(acc_vel->vel_error_xy[0], 2) + powf(acc_vel->vel_error_xy[1], 2));
  if (acc_vel->vel_error_xy[0] != 0 || acc_vel->vel_error_xy[1] != 0) {
    acc_vel->vel_error_rad = atan2f(acc_vel->vel_error_xy[1], acc_vel->vel_error_xy[0]);
  }

  // スカラは使わず、常に最大加速度
  acc_vel->accel[0] = cosf(acc_vel->vel_error_rad) * acc_vel->accel_target;
  acc_vel->accel[1] = sinf(acc_vel->vel_error_rad) * acc_vel->accel_target;
}

void accelBoost(accel_vector_t * acc_vel, target_t * target, bool local_deccel_control_flag)
{
  // これはデバッグ用
  if (local_deccel_control_flag) {
    // バック時は0.7倍
    if (acc_vel->accel[0] < -(acc_vel->accel_target * ACCEL_BACK_SIDE_RATIO)) {
      acc_vel->accel[0] = -(acc_vel->accel_target * ACCEL_BACK_SIDE_RATIO);
    }

    // 減速時に1.8倍
    for (int i = 0; i < 2; i++) {
      if (target->local_vel_now[i] * acc_vel->accel[i] <= 0) {
        acc_vel->accel[i] *= ACCEL_DECCEL_RATIO;
      }

      // 目標座標を追い越した場合、加速度を2倍にして現実の位置に追従
      // 現在座標も速度制御されたタイヤで見ているので、あまりｱﾃにならない
      // これ使える気がする
      /*if ((omni->robot_pos_diff[i] > 0 && output->accel[i] > 0) || (omni->robot_pos_diff[i] < 0 && output->accel[i] < 0)) {
      //output->accel[i] *= 1.5;
    }*/
    }
  }

  // 前後は5m/ss
  // 斜めは2m/ss
  // 
}

void speedControl(accel_vector_t * acc_vel, target_t * target, imu_t * imu)
{
  // 目標速度と差が小さい場合は目標速度をそのまま代入する
  // 目標速度が連続的に変化する場合に適切でないかも
  if (acc_vel->vel_error_scalar <= acc_vel->accel_target / MAIN_LOOP_CYCLE) {
    //convertLocalToGlobal(target->local_vel, target->global_vel_now, imu->yaw_rad);
    target->global_vel_now[0] = (target->local_vel[0]) * cosf(imu->yaw_rad) - (target->local_vel[1]) * sinf(imu->yaw_rad);
    target->global_vel_now[1] = (target->local_vel[0]) * sinf(imu->yaw_rad) + (target->local_vel[1]) * cosf(imu->yaw_rad);

    acc_vel->accel[0] = 0;
    acc_vel->accel[1] = 0;
  }

  // ローカル→グローバル座標系
  // ロボットが回転しても、慣性はグローバル座標系に乗るので、加速度はグローバル座標系に変換してから加算
  // accel
  static float global_vel_diff[2] = {0};
  convertLocalToGlobal(acc_vel->accel, global_vel_diff, imu->yaw_rad);
  target->global_vel_now[0] += global_vel_diff[0] / MAIN_LOOP_CYCLE;
  target->global_vel_now[1] += global_vel_diff[1] / MAIN_LOOP_CYCLE;

  // 次回の計算のためにローカル座標系での速度も更新
  convertGlobalToLocal(target->global_vel_now, target->local_vel_now, imu->yaw_rad);
}
