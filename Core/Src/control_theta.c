#include "control_theta.h"

#include <math.h>
#include <stdint.h>

#include "icm20602_spi.h"
#include "util.h"

// 0.35sで180度反転できる程度
// ave : 3* M_PI (9.42) rad/s  / max : 6 * M_PI(18.84) rad/s
// 0.35sで6*M_PI rpsに達する = 53.8 rad/ss
void thetaControl(RobotCommandV2 * ai_cmd, imu_t * imu, target_t * target)
{
  const float RORATION_RADIAN_PER_SEC_SEC = 8 * 2 * M_PI;  // rev /s^2 -> rad / s^2
  float max_additional_speed_per_cycle = RORATION_RADIAN_PER_SEC_SEC / MAIN_LOOP_CYCLE;

  float angle_diff = getAngleDiff(ai_cmd->target_global_theta, imu->yaw_rad);

  // 目標角度近くでは立ち上がりを緩やかにする
  const float LOW_ACCEL_ZONE_RADIAN = 10 * M_PI / 180;  // deg -> rad
     if (fabsf(angle_diff) < LOW_ACCEL_ZONE_RADIAN) {
    max_additional_speed_per_cycle /= 2;
  }

  // 不感帯設定
  const float DEAD_ZONE_RADIAN = 1 * M_PI / 180;  // deg -> rad
  if (fabsf(angle_diff) < DEAD_ZONE_RADIAN) {
    angle_diff = 0;
  } else {
    if (angle_diff > 0) {
      angle_diff -= DEAD_ZONE_RADIAN;
    } else {
      angle_diff += DEAD_ZONE_RADIAN;
    }
  }

  // 2ax = v^2 -> v = √(2ax)
  float temp_tar_rps = sqrtf(fabsf(2 * angle_diff * RORATION_RADIAN_PER_SEC_SEC));
  temp_tar_rps = copysignf(temp_tar_rps, angle_diff);

  float temp_yaw_rps = clampSize(temp_tar_rps, ai_cmd->angular_velocity_limit);

  // 立ち下がりは目標角度との差によって連続になるが、立ち上がりはそうではないので角加速度で制限する
  float yaw_rps_diff = temp_yaw_rps - target->yaw_rps;
  if (yaw_rps_diff * temp_yaw_rps > 0) {                                                  //符号が同じ場合のみ(=反転時を排除)
    if (fabsf(yaw_rps_diff) > fabsf(target->yaw_rps) + max_additional_speed_per_cycle) {  //加速時のみ適用
      yaw_rps_diff = clampSize(yaw_rps_diff, max_additional_speed_per_cycle);
    }
  }
  target->yaw_rps += yaw_rps_diff;

  // 旋回方向のみの減衰項
  const float YAW_IMU_DRAG_GAIN = 40;  // 旋回減衰(:IMU) 80だと振動する
  float rad_per_sec = (imu->yaw_rad - imu->pre_yaw_rad) * MAIN_LOOP_CYCLE;
  target->yaw_rps_drag = -YAW_IMU_DRAG_GAIN * (rad_per_sec - target->yaw_rps);
}

// 静止中に一気にvision角度を合わせるやつ
static void followVisionAngleInStop(imu_t * imu, RobotCommandV2 * ai_cmd, connection_t * conn)
{
  static uint32_t yaw_update_cnt = 0;
  imu->yaw_deg_diff_integral += fabsf(imu->pre_yaw_deg - imu->yaw_deg);
  yaw_update_cnt++;
  if (yaw_update_cnt > MAIN_LOOP_CYCLE / 2) {  // 2Hz
    yaw_update_cnt = 0;
    if (imu->yaw_deg_diff_integral < 1) {
      // 機体が旋回していないとき
      imu->theta_override_flag = true;

      // visionとの角度差があるときにアプデ
      if (conn->connected_ai && ai_cmd->is_vision_available && getAngleDiff(imu->yaw_deg, ai_cmd->vision_global_theta) > 10 * M_PI / 180) {
        imu->yaw_deg = ai_cmd->vision_global_theta * 180 / M_PI;
      }
    } else {
      imu->theta_override_flag = false;
    }
    imu->yaw_deg_diff_integral = 0;
  }

  imu->pre_yaw_deg = imu->yaw_deg;
}

// vision NG -> ONになったときに強制更新するやつ
static void followVisionAngleInWakeUp(imu_t * imu, RobotCommandV2 * ai_cmd, connection_t * conn)
{
  static int vision_lost_cycle_cnt = 0;
  if (ai_cmd->is_vision_available) {
    if (vision_lost_cycle_cnt >= MAIN_LOOP_CYCLE) {
      // 今は無効化してる
      // imu->yaw_deg = ai_cmd->vision_global_theta * 180 / M_PI;
    }
    vision_lost_cycle_cnt = 0;
  } else if (vision_lost_cycle_cnt <= MAIN_LOOP_CYCLE) {
    vision_lost_cycle_cnt++;
  }
}

static void imuYawComplementFillter(system_t * sys, debug_t * debug, imu_t * imu, RobotCommandV2 * ai_cmd)
{
  if (sys->main_mode == MAIN_MODE_CMD_DEBUG_MODE && !ai_cmd->is_vision_available) {
    // デバッグ用、visionないときはtargetへ補正する
    imu->yaw_deg = imu->yaw_deg - (getAngleDiff(imu->yaw_deg * M_PI / 180.0, ai_cmd->target_global_theta) * 180.0 / M_PI) * 0.001;  // 0.001 : gain

  } else if (!ai_cmd->is_vision_available || debug->latency_check.enabled) {
    // VisionLost時は補正しない
    // レイテンシチェック中(一定速度での旋回中)は相補フィルタ切る

  } else {
    imu->yaw_deg = imu->yaw_deg - (getAngleDiff(imu->yaw_deg * M_PI / 180.0, ai_cmd->vision_global_theta) * 180.0 / M_PI) * 0.001;  // 0.001 : gain
  }

  imu->pre_yaw_rad = imu->yaw_rad;
  imu->yaw_rad = imu->yaw_deg * M_PI / 180;
}

void yawFilter(system_t * sys, debug_t * debug, imu_t * imu, RobotCommandV2 * ai_cmd, connection_t * conn)
{
  followVisionAngleInStop(imu, ai_cmd, conn);
  followVisionAngleInWakeUp(imu, ai_cmd, conn);

  ICM20602_read_IMU_data((float)1.0 / MAIN_LOOP_CYCLE, &(imu->yaw_deg));

  imuYawComplementFillter(sys, debug, imu, ai_cmd);
}
