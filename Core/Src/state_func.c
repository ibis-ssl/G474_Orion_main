#include <state_func.h>

#include "actuator.h"
#include "can_ibis.h"
#include "control_omni_angle.h"
#include "control_speed.h"
#include "control_theta.h"
#include "omni_wheel.h"
#include "robot_control.h"
#include "util.h"

void motorTest(system_t * sys, output_t * output, omni_t * omni)
{
  const float OUT_LIMIT_TEST = 40.0;
  const float OUT_MOVE_VEL = 2.0;
  const float OUT_SPIN_OMG = 20;
  if (swForwardPushed(sys->sw_adc_raw)) {
    output->velocity[0] = OUT_MOVE_VEL;
    output->velocity[1] = 0;
    output->omega = 0;
    omniMove(output, OUT_LIMIT_TEST);  // fwd
    setHighEventLED();
  } else if (swBackPushed(sys->sw_adc_raw)) {
    output->velocity[0] = -OUT_MOVE_VEL;
    output->velocity[1] = 0;
    output->omega = 0;
    omniMove(output, OUT_LIMIT_TEST);  // back
    setHighEventLED();
  } else if (swLeftPushed(sys->sw_adc_raw)) {
    output->velocity[0] = 0;
    output->velocity[1] = OUT_MOVE_VEL;
    output->omega = 0;
    omniMove(output, OUT_LIMIT_TEST);  // left
    setHighEventLED();
  } else if (swRightPushed(sys->sw_adc_raw)) {
    output->velocity[0] = 0;
    output->velocity[1] = -OUT_MOVE_VEL;
    output->omega = 0;
    omniMove(output, OUT_LIMIT_TEST);  // right
    setHighEventLED();
  } else if (swCentorPushed(sys->sw_adc_raw)) {
    output->velocity[0] = 0;
    output->velocity[1] = 0;
    output->omega = OUT_SPIN_OMG;
    omniMove(output, OUT_LIMIT_TEST);  // spin
    setHighEventLED();
  } else {
    output->velocity[0] = 0;
    output->velocity[1] = 0;
    output->omega = 0;
    omniStopAll(output);
    setLowEventLED();
  }
  actuator_motor5(0.0);
}

void dribblerTest(system_t * sys, output_t * output, can_raw_t * can_raw)
{
  if (swCentorPushed(sys->sw_adc_raw)) {
    if (can_raw->ball_detection[0]) {
      actuator_motor5(0.5);

    } else {
      actuator_motor5(0.20);
    }
    setHighEventLED();
  } else {
    actuator_motor5(0.0);
    setLowEventLED();
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

  if (dribbler_up == false && swRightPushed(sys->sw_adc_raw)) {
    dribbler_up = true;
    actuator_dribbler_down();
  } else if (dribbler_up == true && swLeftPushed(sys->sw_adc_raw)) {
    dribbler_up = false;
    actuator_dribbler_up();
  }

  if (swCentorPushed(sys->sw_adc_raw)) {
    if (!manual_mode) {
      actuator_motor5(0.5);
    }
    setHighEventLED();
    if (can_raw->ball_detection[0] == 1 || manual_mode) {
      if (sys->kick_state == 0) {
        kicker_select_straight();
        kicker_kick_start(0.2);
        sys->kick_state = 1;
      }
    }
  } else if (swBackPushed(sys->sw_adc_raw)) {
    if (!manual_mode) {
      actuator_motor5(0.5);
    }
    setHighEventLED();
    if (can_raw->ball_detection[0] == 1 || manual_mode) {
      if (sys->kick_state == 0) {
        kicker_select_chip();
        kicker_kick_start(0.5);
        sys->kick_state = 1;
      }
    }
  } else {
    actuator_motor5(0.0);
    setLowEventLED();
    kicker_charge_start();
    actuator_kicker_cmd_voltage(400.0);
  }
  omniStopAll(output);
}

static float getTargetThetaInLatencyCheckMode(debug_t * debug, RobotCommandV2 * ai_cmd)
{
  if (!debug->latency_check.enabled) {
    return ai_cmd->target_global_theta;  // そもそも呼ばれないようにするべきだが、安全なのはこれ
  }

  if (debug->latency_check.seq_cnt > 0) {
    debug->latency_check.seq_cnt--;
  } else {
    // 終わった瞬間吹っ飛ぶので、指令値近くなったときに停止
    if (getAngleDiff(ai_cmd->target_global_theta, debug->latency_check.rotation_target_theta) < 0.1) {
      debug->latency_check.enabled = false;
    }
    // complete!!
  }
  return debug->latency_check.rotation_target_theta + (float)1 / MAIN_LOOP_CYCLE;  // 1 rad/s
}

void latencyCheck(system_t * sys, debug_t * debug, RobotCommandV2 * ai_cmd, output_t * output, imu_t * imu)
{
  const float OMEGA_GAIN_KP = 160.0;
  const float OMEGA_GAIN_KD = 4000.0;
  const float OUTPUT_OMEGA_LIMIT = 10.0;  // ~ rad/s

  if (debug->latency_check.enabled) {
    debug->latency_check.rotation_target_theta = getTargetThetaInLatencyCheckMode(debug, ai_cmd);
    output->omega = (getAngleDiff(debug->latency_check.rotation_target_theta, imu->yaw_rad) * OMEGA_GAIN_KP) - (getAngleDiff(imu->yaw_rad, imu->pre_yaw_rad) * OMEGA_GAIN_KD);

    output->omega = clampSize(output->omega, OUTPUT_OMEGA_LIMIT);
    output->velocity[0] = 0;
    output->velocity[1] = 0;

    omniMove(output, 10);
    return;
  } else {
    omniStopAll(output);
  }

  if (!swCentorPushed(sys->sw_adc_raw)) {
    debug->latency_check.seq_cnt = 0;
    return;
  }

  debug->latency_check.seq_cnt++;
  if (debug->latency_check.seq_cnt > MAIN_LOOP_CYCLE) {
    debug->latency_check.enabled = true;
    debug->latency_check.seq_cnt = MAIN_LOOP_CYCLE * 10;
    debug->latency_check.rotation_target_theta = ai_cmd->target_global_theta;
  }
}

void motorCalibration(system_t * sys)
{
  static uint32_t calib_start_cnt = 0;
  if (swRightPushed(sys->sw_adc_raw)) {
    calib_start_cnt++;
    if (calib_start_cnt > 1000) {
      actuator_motor_calib(0);
    }
  } else if (swLeftPushed(sys->sw_adc_raw)) {
    calib_start_cnt++;
    if (calib_start_cnt > 1000) {
      actuator_motor_calib(1);
    }
  } else {
    calib_start_cnt = 0;
  }
}

void manualPowerReset(system_t * sys)
{
  static int sw_push_cnt = 0;
  if (swCentorPushed(sys->sw_adc_raw)) {
    sw_push_cnt++;
  } else {
    sw_push_cnt = 0;
  }
  if (sw_push_cnt > MAIN_LOOP_CYCLE / 2) {
    sw_push_cnt = 0;
    resetPowerBoard();
  }
}

void maintaskRun(
  system_t * sys, RobotCommandV2 * ai_cmd, imu_t * imu, accel_vector_t * acc_vel, integ_control_t * integ, target_t * target, omni_t * omni, mouse_t * mouse, debug_t * debug, output_t * output,
  can_raw_t * can_raw, motor_t * motor, camera_t * cam)

{
  const float OMNI_OUTPUT_LIMIT = 40;
  // 上げると過電流エラーになりがち｡
  // 速度制限にはrobotControlのOUTPUT_XY_LIMITを使用する｡

  // デバッガ接続用にエラー入れる
  if (swCentorPushed(sys->sw_adc_raw)) {
    sys->error_flag = true;
  }

  if (sys->stop_flag) {
    clearSpeedContrlValue(acc_vel, target, imu, omni, motor);
  }

  bool local_deccel_control_flag = false;
  if (sys->main_mode == MAIN_MODE_MANUAL_CONTROL) {
    local_deccel_control_flag = true;
  }

  setLocalTargetSpeed(ai_cmd, target, imu);

  if (!local_deccel_control_flag) {
  } else {
    if (cam->is_detected) {
      // 320x240
      //target->yaw_rps = (float)(cam->pos_xy[0] - 160) / 10;
      target->local_vel[1] = (float)(cam->pos_xy[0] - 160) / 100;
    } else {
      target->local_vel[1] = 0;
    }
  }

  setTargetAccel(ai_cmd, acc_vel, local_deccel_control_flag);

  accelControl(acc_vel, target, local_deccel_control_flag);
  speedControl(acc_vel, target, imu);

  thetaControl(ai_cmd, imu, target);

  setTargetOmniAngle(target);
  omniAngleControl(target, output, motor);

  // いまのところvision lostしたら止める
  if (sys->main_mode == MAIN_MODE_CMD_DEBUG_MODE) {
    omniStopAll(output);
    //デバッグ用に出力しないだけなのでクリアはしない
  } else if (sys->stop_flag || ai_cmd->stop_emergency || !ai_cmd->is_vision_available || ai_cmd->elapsed_time_ms_since_last_vision > 500) {
    omniStopAll(output);
  } else {
    omniMoveIndiv(output, OMNI_OUTPUT_LIMIT);
  }

  sendActuatorCanCmdRun(ai_cmd, sys, can_raw);
}

void maintaskStop(output_t * output)
{
  omniStopAll(output);
  sendActuatorCanCmdStop();
}