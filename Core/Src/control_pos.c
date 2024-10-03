#include "control_pos.h"

#include "util.h"

// 加速度パラメーター
#define ACCEL_LIMIT (6)       // m/ss
#define ACCEL_LIMIT_BACK (4)  // m/ss

#define ACCEL_TO_OUTPUT_GAIN (0.5)

// 減速方向は制動力を増強 1.5～2.0
#define DEC_BOOST_GAIN (1.0)

void localPositionFeedback(integration_control_t * integ, imu_t * imu, target_t * target, RobotCommandV2 * ai_cmd, omni_t * omni, mouse_t * mouse, accel_vector_t * acc_vel, output_t * output)
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
  target->pos_ctrl.target_pos_dist_scalar = calcScalar(integ->position_diff[0], integ->position_diff[1]);

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
  target->pos_ctrl.target_scalar_vel = pow(2 * (ACCEL_LIMIT * DEC_BOOST_GAIN * 0.75) * target->pos_ctrl.target_pos_dist_scalar, 0.5) + ai_cmd->mode_args.position.terminal_velocity;

  // v^2 = 2 * acc * 2 * 0.1
  // デバッグ用の1m/s｡本当はai_cmdの値を使う
  float linear_velocity_limit = ai_cmd->linear_velocity_limit;
  //float linear_velocity_limit = 1.5;
  if (target->pos_ctrl.target_scalar_vel > linear_velocity_limit) {
    target->pos_ctrl.to_stop_mode_flag = false;
  } else {
    target->pos_ctrl.to_stop_mode_flag = true;
  }
  target->pos_ctrl.target_scalar_vel = clampSize(target->pos_ctrl.target_scalar_vel, linear_velocity_limit);
  // 目標地点付近での制御は別で用意していいかも
  if (target->pos_ctrl.target_pos_dist_scalar < 0.03) {
    target->pos_ctrl.target_scalar_vel = 0;
  }

  target->pos_ctrl.target_vel_angle = atan2(target->global_vel[1], target->global_vel[0]);
  //float dummy_speed[2] = {1, 0};
  //convertLocalToGlobal(dummy_speed, target->pos_ctrl.global_odom_speed, imu->yaw_angle_rad);
  convertLocalToGlobal(using_omni_speed, target->pos_ctrl.global_odom_speed, imu->yaw_angle_rad);

  // 関数名と違うけどターゲット速度座標系に変換
  convertGlobalToLocal(target->pos_ctrl.global_odom_speed, target->pos_ctrl.current_speed_crd, target->pos_ctrl.target_vel_angle);
  target->pos_ctrl.target_crd_acc[0] = (target->pos_ctrl.target_scalar_vel - target->pos_ctrl.current_speed_crd[0]) * 7.5;  //程々に下げたほうがいいがち
  target->pos_ctrl.target_crd_acc[1] = -target->pos_ctrl.current_speed_crd[1] * 5;                                          //20だと発振

  if (target->pos_ctrl.target_crd_acc[0] > ACCEL_LIMIT) {
    target->pos_ctrl.target_crd_acc[0] = ACCEL_LIMIT;
  } else if (target->pos_ctrl.target_crd_acc[0] < -ACCEL_LIMIT * DEC_BOOST_GAIN) {
    target->pos_ctrl.target_crd_acc[0] = -ACCEL_LIMIT * DEC_BOOST_GAIN;  // 減速方向のみ2倍
  }
  if (!target->pos_ctrl.to_stop_mode_flag && target->pos_ctrl.target_crd_acc[0] < 0) {  //速度制限への追従は控えめ
    target->pos_ctrl.target_crd_acc[0] /= 10;
  }
  target->pos_ctrl.target_crd_acc[1] = clampSize(target->pos_ctrl.target_crd_acc[1], ACCEL_LIMIT * DEC_BOOST_GAIN);  // 常に減速のため､2倍

  convertLocalToGlobal(target->pos_ctrl.target_crd_acc, target->pos_ctrl.global_acc, target->pos_ctrl.target_vel_angle);
  convertGlobalToLocal(target->pos_ctrl.global_acc, output->accel, imu->yaw_angle_rad);

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

  /*if (target->pos_ctrl.target_scalar_vel == 0 && target->pos_ctrl.current_speed_crd[0] < 0.1 && target->pos_ctrl.current_speed_crd[0] < 0.1) {
    output->velocity[0] = 0;
    output->velocity[1] = 0;
  }*/
}
