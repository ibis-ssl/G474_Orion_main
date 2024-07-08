#include "test_func.h"

#include "actuator.h"
#include "omni_wheel.h"
#include "util.h"

bool isLatencyCheckModeEnabled(system_t * sys, debug_t * debug, RobotCommandV2 * ai_cmd)
{
  if (debug->latency_check_enabled) {
    return true;
  }

  if (!swCentorPushed(sys->sw_data)) {
    debug->latency_check_seq_cnt = 0;
    return false;
  }

  debug->latency_check_seq_cnt++;
  if (debug->latency_check_seq_cnt > MAIN_LOOP_CYCLE) {
    debug->latency_check_enabled = true;
    debug->latency_check_seq_cnt = MAIN_LOOP_CYCLE * 10;
    debug->rotation_target_theta = ai_cmd->target_global_theta;
  }
  return debug->latency_check_enabled;
}

float getTargetThetaInLatencyCheckMode(debug_t * debug, RobotCommandV2 * ai_cmd)
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

void motorTest(system_t * sys, output_t * output)
{
  if (swForwardPushed(sys->sw_data)) {
    omniMove(4.0, 0.0, 0.0, 4.0, output);  // fwd
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (swBackPushed(sys->sw_data)) {
    omniMove(-4.0, 0.0, 0.0, 4.0, output);  // back
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (swLeftPushed(sys->sw_data)) {
    omniMove(0.0, -4.0, 0.0, 4.0, output);  // left
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (swRightPushed(sys->sw_data)) {
    omniMove(0.0, 4.0, 0.0, 4.0, output);  // right
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (swCentorPushed(sys->sw_data)) {
    omniMove(0.0, 0.0, 20.0, 4.0, output);  // spin
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else {
    omniMove(0.0, 0.0, 0.0, 0.0, output);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
  }
  actuator_motor5(0.0, 0.0);
}

void dribblerTest(system_t * sys, output_t * output)
{
  if (swCentorPushed(sys->sw_data)) {
    actuator_motor5(0.5, 1.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else {
    actuator_motor5(0.0, 0.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
  }
  omniMove(0.0, 0.0, 0.0, 0.0, output);
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
      actuator_motor5(0.5, 1.0);
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
      actuator_motor5(0.5, 1.0);
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
    actuator_motor5(0.0, 0.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
    actuator_kicker(1, 1);  // charge enable
    actuator_kicker_voltage(400.0);
  }
  omniMove(0.0, 0.0, 0.0, 0.0, output);
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