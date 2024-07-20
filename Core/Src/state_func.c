#include <state_func.h>

#include "actuator.h"
#include "can_ibis.h"
#include "omni_wheel.h"
#include "robot_control.h"
#include "util.h"

void motorTest(system_t * sys, output_t * output)
{
  const float OUT_LIMIT_TEST = 20.0;
  const float OUT_MOVE_VEL = 2.0;
  const float OUT_SPIN_OMG = 20;
  if (swForwardPushed(sys->sw_data)) {
    output->velocity[0] = OUT_MOVE_VEL;
    output->velocity[1] = 0;
    output->omega = 0;
    omniMove(output, OUT_LIMIT_TEST);  // fwd
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (swBackPushed(sys->sw_data)) {
    output->velocity[0] = -OUT_MOVE_VEL;
    output->velocity[1] = 0;
    output->omega = 0;
    omniMove(output, OUT_LIMIT_TEST);  // back
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (swLeftPushed(sys->sw_data)) {
    output->velocity[0] = 0;
    output->velocity[1] = OUT_MOVE_VEL;
    output->omega = 0;
    omniMove(output, OUT_LIMIT_TEST);  // left
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (swRightPushed(sys->sw_data)) {
    output->velocity[0] = 0;
    output->velocity[1] = -OUT_MOVE_VEL;
    output->omega = 0;
    omniMove(output, OUT_LIMIT_TEST);  // right
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (swCentorPushed(sys->sw_data)) {
    output->velocity[0] = 0;
    output->velocity[1] = 0;
    output->omega = OUT_SPIN_OMG;
    omniMove(output, OUT_LIMIT_TEST);  // spin
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else {
    output->velocity[0] = 0;
    output->velocity[1] = 0;
    output->omega = 0;
    omniStopAll(output);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
  }
  actuator_motor5(0.0);
}

void dribblerTest(system_t * sys, output_t * output, can_raw_t * can_raw)
{
  if (swCentorPushed(sys->sw_data)) {
    if (can_raw->ball_detection[0]) {
      actuator_motor5(0.5);

    } else {
      actuator_motor5(0.20);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else {
    actuator_motor5(0.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
  }
  omniStopAll(output);
}

void kickerTest(system_t * sys, can_raw_t * can_raw, bool manual_mode, output_t * output)
{
  static bool dribbler_up = false;

  if (sys->kick_state != 0) {
    if (sys->kick_state > MAIN_LOOP_CYCLE / 2) {
      if (can_raw->ball_detection[0] == 0) {
        sys->kick_state = 0;
      }
    } else {
      sys->kick_state++;
    }
  }

  if (dribbler_up == false && swRightPushed(sys->sw_data)) {
    dribbler_up = true;
    actuator_dribbler_down();
  } else if (dribbler_up == true && swLeftPushed(sys->sw_data)) {
    dribbler_up = false;
    actuator_dribbler_up();
  }

  if (swCentorPushed(sys->sw_data)) {
    if (!manual_mode) {
      actuator_motor5(0.5);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
    if (can_raw->ball_detection[0] == 1 || manual_mode) {
      if (sys->kick_state == 0) {
        actuator_kicker(2, 0);  // straight
        actuator_kicker(3, 50);
        //actuator_kicker(3, 100);
        sys->kick_state = 1;
      }
    }
  } else if (swBackPushed(sys->sw_data)) {
    if (!manual_mode) {
      actuator_motor5(0.5);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
    if (can_raw->ball_detection[0] == 1 || manual_mode) {
      if (sys->kick_state == 0) {
        actuator_kicker(2, 1);  // chip
        actuator_kicker(3, 100);
        //actuator_kicker(3, 255);
        sys->kick_state = 1;
      }
    }
  } else {
    actuator_motor5(0.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
    actuator_kicker(1, 1);  // charge enable
    actuator_kicker_voltage(400.0);
  }
  omniStopAll(output);
}

static float getTargetThetaInLatencyCheckMode(debug_t * debug, RobotCommandV2 * ai_cmd)
{
  if (!debug->latency_check_enabled) {
    return ai_cmd->target_global_theta;  // そもそも呼ばれないようにするべきだが、安全なのはこれ
  }

  if (debug->latency_check_seq_cnt > 0) {
    debug->latency_check_seq_cnt--;
  } else {
    // 終わった瞬間吹っ飛ぶので、指令値近くなったときに停止
    if (getAngleDiff(ai_cmd->target_global_theta, debug->rotation_target_theta) < 0.1) {
      debug->latency_check_enabled = false;
    }
    // complete!!
  }
  return debug->rotation_target_theta + (float)1 / MAIN_LOOP_CYCLE;  // 1 rad/s
}

void latencyCheck(system_t * sys, debug_t * debug, RobotCommandV2 * ai_cmd, output_t * output, imu_t * imu)
{
  const float OMEGA_GAIN_KP = 160.0;
  const float OMEGA_GAIN_KD = 4000.0;
  const float OUTPUT_OMEGA_LIMIT = 10.0;  // ~ rad/s

  if (debug->latency_check_enabled) {
    debug->rotation_target_theta = getTargetThetaInLatencyCheckMode(debug, ai_cmd);
    output->omega = (getAngleDiff(debug->rotation_target_theta, imu->yaw_angle_rad) * OMEGA_GAIN_KP) - (getAngleDiff(imu->yaw_angle_rad, imu->pre_yaw_angle_rad) * OMEGA_GAIN_KD);

    output->omega = clampSize(output->omega, OUTPUT_OMEGA_LIMIT);
    output->velocity[0] = 0;
    output->velocity[1] = 0;

    omniMove(output, 10);
    return;
  } else {
    omniStopAll(output);
  }

  if (!swCentorPushed(sys->sw_data)) {
    debug->latency_check_seq_cnt = 0;
    return;
  }

  debug->latency_check_seq_cnt++;
  if (debug->latency_check_seq_cnt > MAIN_LOOP_CYCLE) {
    debug->latency_check_enabled = true;
    debug->latency_check_seq_cnt = MAIN_LOOP_CYCLE * 10;
    debug->rotation_target_theta = ai_cmd->target_global_theta;
  }
}

void motorCalibration(system_t * sys)
{
  static uint32_t calib_start_cnt = 0;
  if (swRightPushed(sys->sw_data)) {
    calib_start_cnt++;
    if (calib_start_cnt > 1000) {
      actuator_motor_calib(0);
    }
  } else if (swLeftPushed(sys->sw_data)) {
    calib_start_cnt++;
    if (calib_start_cnt > 1000) {
      actuator_motor_calib(1);
    }
  } else {
    calib_start_cnt = 0;
  }
}

void maintaskRun(
  system_t * sys, RobotCommandV2 * ai_cmd, imu_t * imu, accel_vector_t * acc_vel, integration_control_t * integ, target_t * target, omni_t * omni, mouse_t * mouse, debug_t * debug, output_t * output,
  can_raw_t * can_raw)

{
  const float OMNI_OUTPUT_LIMIT = 40;  //上げると過電流エラーになりがち

  // 全部で250us
  robotControl(sys, ai_cmd, imu, acc_vel, integ, target, omni, mouse, debug, output);

  // いまのところvision lostしたら止める
  if (ai_cmd->stop_emergency || !ai_cmd->is_vision_available) {
    //resetLocalSpeedControl(&ai_cmd);
    omniStopAll(output);
  } else {
    omniMove(output, OMNI_OUTPUT_LIMIT);
  }

  sendActuatorCanCmdRun(ai_cmd, sys, can_raw);
}

void maintaskStop(output_t * output)
{
  omniStopAll(output);
  sendActuatorCanCmdStop();
}