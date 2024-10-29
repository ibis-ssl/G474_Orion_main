#include "control_omni_angle.h"

#include "math.h"
#include "util.h"

static float rotation_length_omni = OMNI_DIAMETER * M_PI;
static const float sinM1 = sin(30 * M_PI / 180);
static const float cosM1 = cos(30 * M_PI / 180);

static const float sinM2 = sin(315 * M_PI / 180);
static const float cosM2 = cos(315 * M_PI / 180);

static const float sinM3 = sin(225 * M_PI / 180);
static const float cosM3 = cos(225 * M_PI / 180);

static const float sinM4 = sin(150 * M_PI / 180);
static const float cosM4 = cos(150 * M_PI / 180);

void setTargetOmniAngle(target_t * target)
{
  float rotation_omega_motor = 0;

  //rotation_omega_motor = ROBOT_RADIUS * target->yaw_rps;

  target->omni_angle[0].current_rps = ((target->local_vel_now[1] * sinM1) + (target->local_vel_now[0] * cosM1) + rotation_omega_motor) / rotation_length_omni;
  target->omni_angle[1].current_rps = ((target->local_vel_now[1] * sinM2) + (target->local_vel_now[0] * cosM2) + rotation_omega_motor) / rotation_length_omni;
  target->omni_angle[2].current_rps = ((target->local_vel_now[1] * sinM3) + (target->local_vel_now[0] * cosM3) + rotation_omega_motor) / rotation_length_omni;
  target->omni_angle[3].current_rps = ((target->local_vel_now[1] * sinM4) + (target->local_vel_now[0] * cosM4) + rotation_omega_motor) / rotation_length_omni;

  for (int i = 0; i < 4; i++) {
    target->omni_angle[i].angle += target->omni_angle[i].current_rps;
  }
}

void omniAngleControl(target_t * target, output_t * output, motor_t * motor)
{
  static const float ANGLE_KP = 10;
  static const float ANGLE_KD = 10;
  for (int i = 0; i < 4; i++) {
    target->omni_angle[i].diff = target->omni_angle[i].angle - motor->enc_angle[i];
    target->omni_angle[i].real_rps = motor->rps[i];

    output->motor_voltage[i] = target->omni_angle[i].diff * ANGLE_KP - target->omni_angle[i].real_rps * ANGLE_KD;
  }
}