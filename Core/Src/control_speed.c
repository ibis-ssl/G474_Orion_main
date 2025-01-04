
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

void setTargetAccel(RobotCommandV2 * ai_cmd, accel_vector_t * acc_vel, bool local_devvel_control_flag)
{
  if (local_devvel_control_flag) {
    acc_vel->accel_target = 5.0;
    return;
  }

  if (ai_cmd->acceleration_limit < 3.0 || ai_cmd->acceleration_limit > 20) {
    acc_vel->accel_target = 3.0;
  } else {
    acc_vel->accel_target = ai_cmd->acceleration_limit;
  }
}

void accelControl(accel_vector_t * acc_vel, target_t * target, bool local_devvel_control_flag)
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

  // バック方向だけ加速度制限
  if (acc_vel->accel[0] < -(acc_vel->accel_target * ACCEL_BACK_SIDE_MULTI)) {
    acc_vel->accel[0] = -(acc_vel->accel_target * ACCEL_BACK_SIDE_MULTI);
  }

  // 減速時に1.8倍
  // いずれ指令値側で増やすので、消す
  if (local_devvel_control_flag) {
    for (int i = 0; i < 2; i++) {
      if (target->local_vel_now[i] * acc_vel->accel[i] <= 0) {
        acc_vel->accel[i] *= ACCEL_DECCEL_MULTI;
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
