#include "control_theta.h"

#include "icm20602_spi.h"
#include "util.h"

void thetaControl(RobotCommandV2 * ai_cmd, imu_t * imu, target_t * target)
{
  const float DIFF_TO_RPS_GAIN = 2.0;  // rad -> rad/s
  const float DEAD_ZONE_DEG = 5;       // rad -> rad/s

  float angle_diff = getAngleDiff(ai_cmd->target_global_theta, imu->yaw_rad);

  if (fabs(angle_diff) < DEAD_ZONE_DEG * M_PI / 180) {
    angle_diff = 0;
  }

  float temp_tar_rps = angle_diff * DIFF_TO_RPS_GAIN;
  target->yaw_rps = clampSize(temp_tar_rps, ai_cmd->angular_velocity_limit / MAIN_LOOP_CYCLE);
}

// 静止中に一気にvision角度を合わせるやつ
static void followVisionAngleInStop(imu_t * imu, RobotCommandV2 * ai_cmd, connection_t * conn)
{
  static uint32_t yaw_update_cnt = 0;
  imu->yaw_deg_diff_integral += fabs(imu->pre_yaw_deg - imu->yaw_deg);
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
