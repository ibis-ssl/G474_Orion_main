#include "control_theta.h"

#include "util.h"

//#define OMEGA_GAIN_KP (160.0)
#define OMEGA_GAIN_KP (30.0)
#define OMEGA_GAIN_KD (4.0 * MAIN_LOOP_CYCLE)

void thetaControl(RobotCommandV2 * ai_cmd, output_t * output, imu_t * imu, omega_target_t * omega_target)
{
  const float OUTPUT_OMEGA_LIMIT = 20.0;  // ~ rad/s

  float target_diff_angle = getAngleDiff(ai_cmd->target_global_theta, omega_target->current_target);
  target_diff_angle = clampSize(target_diff_angle, ai_cmd->angular_velocity_limit / MAIN_LOOP_CYCLE);
  omega_target->current_target += target_diff_angle;
  // PID
  output->omega = (getAngleDiff(omega_target->current_target, imu->yaw_rad) * OMEGA_GAIN_KP) - (getAngleDiff(imu->yaw_rad, imu->pre_yaw_rad) * OMEGA_GAIN_KD);
  output->omega = clampSize(output->omega, OUTPUT_OMEGA_LIMIT);
  //output->omega = 0;
}