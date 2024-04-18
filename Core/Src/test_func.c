#include "test_func.h"

void motor_test(system_t * sys)
{
  if (decode_SW(sys->sw_data) & 0b00000001) {
    omni_move(4.0, 0.0, 0.0, 4.0);  // fwd
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (decode_SW(sys->sw_data) & 0b00000010) {
    omni_move(-4.0, 0.0, 0.0, 4.0);  // back
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (decode_SW(sys->sw_data) & 0b00000100) {
    omni_move(0.0, -4.0, 0.0, 4.0);  // left
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (decode_SW(sys->sw_data) & 0b00001000) {
    omni_move(0.0, 4.0, 0.0, 4.0);  // right
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (decode_SW(sys->sw_data) & 0b00010000) {
    omni_move(0.0, 0.0, 20.0, 4.0);  // spin
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else {
    omni_move(0.0, 0.0, 0.0, 0.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
  }
  actuator_motor5(0.0, 0.0);
}

void dribbler_test(system_t * sys)
{
  if (decode_SW(sys->sw_data) & 0b00010000) {
    actuator_motor5(0.5, 1.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else {
    actuator_motor5(0.0, 0.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
  }
  omni_move(0.0, 0.0, 0.0, 0.0);
}

void kicker_test(system_t * sys, can_raw_t * can_raw, bool manual_mode)
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

  if (dribbler_up == false && decode_SW(sys->sw_data) & 0b00000100) {
    dribbler_up = true;
    actuator_dribbler_down();
  } else if (dribbler_up == true && decode_SW(sys->sw_data) & 0b00001000) {
    dribbler_up = false;
    actuator_dribbler_up();
  }

  if (decode_SW(sys->sw_data) & 0b00010000) {
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
  } else if (decode_SW(sys->sw_data) & 0b00000010) {
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
    actuator_kicker_voltage(300.0);
  }
  omni_move(0.0, 0.0, 0.0, 0.0);
}

void motor_calibration(system_t * sys)
{
  static uint32_t calib_start_cnt = 0;
  if (decode_SW(sys->sw_data) & 0b00000100) {
    calib_start_cnt++;
    if (calib_start_cnt > 1000) {
      actuator_motor_calib(0);
    }
  } else if (decode_SW(sys->sw_data) & 0b00001000) {
    calib_start_cnt++;
    if (calib_start_cnt > 1000) {
      actuator_motor_calib(1);
    }
  } else {
    calib_start_cnt = 0;
  }
}