/*
 * robot_control.c
 *
 *  Created on: May 19, 2024
 *      Author: hiroyuki
 */
#include "robot_control.h"

#include "robot_packet.h"
#include "util.h"

// スカラ速度制限
#define SPEED_SCALAR_LIMIT (7.0)

void setLocalTargetSpeed(RobotCommandV2 * ai_cmd, target_t * target, imu_t * imu)
{
  if (ai_cmd->control_mode == SIMPLE_VELOCITY_TARGET_MODE) {
    target->global_vel[0] = ai_cmd->mode_args.simple_velocity.target_global_vel[0];
    target->global_vel[1] = ai_cmd->mode_args.simple_velocity.target_global_vel[1];
    clampScalarSize(target->global_vel, SPEED_SCALAR_LIMIT);

    convertGlobalToLocal(target->global_vel, target->local_vel, imu->yaw_rad);

  } else if (ai_cmd->control_mode == POLAR_VELOCITY_TARGET_MODE) {
    float temp_gloval_vel[2];
    temp_gloval_vel[0] = ai_cmd->mode_args.polar_velocity.target_global_velocity_r * cosf(ai_cmd->mode_args.polar_velocity.target_global_velocity_theta);
    temp_gloval_vel[1] = ai_cmd->mode_args.polar_velocity.target_global_velocity_r * sinf(ai_cmd->mode_args.polar_velocity.target_global_velocity_theta);

    convertGlobalToLocal(temp_gloval_vel, target->local_vel, imu->yaw_rad);

  } else {
    target->local_vel[0] = 0;
    target->local_vel[1] = 0;
  }
}

void setOutZero(output_t * output)
{
  output->velocity[0] = 0;
  output->velocity[1] = 0;
  output->omega = 0;
}

void clearSpeedContrlValue(accel_vector_t * acc_vel, target_t * target, imu_t * imu, omni_t * omni, motor_t * motor)
{
  acc_vel->vel_error_rad = 0;
  acc_vel->vel_error_scalar = 0;
  for (int i = 0; i < 2; i++) {
    acc_vel->vel_error_xy[i] = 0;  //毎回更新するので多分いらない
    //target->local_vel_now[i] = omni->local_odom_speed_mvf[i];
    target->global_pos[i] = omni->odom[i];
  }

  for (int i = 0; i < 4; i++) {
    target->omni_angle[i].angle_rad = motor->angle_rad[i];
  }
  convertLocalToGlobal(omni->local_odom_speed_mvf, target->global_vel_now, imu->yaw_rad);
}
