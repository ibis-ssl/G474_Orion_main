/*
 * actuator.c
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */
#include "actuator.h"

#include "can_ibis.h"
#include "util.h"

static void motor_cmd_can1(uint16_t motor_id, float duty)
{
  uint8_t senddata_motor[8] = {0};
  float_to_uchar4(senddata_motor, duty);
  can1_send(motor_id, senddata_motor);
}
static void motor_cmd_can2(uint16_t motor_id, float duty)
{
  uint8_t senddata_motor[8] = {0};
  float_to_uchar4(senddata_motor, duty);
  can2_send(motor_id, senddata_motor);
}

void actuator_motor1(float duty) { motor_cmd_can1(0x100, duty); }
void actuator_motor2(float duty) { motor_cmd_can1(0x101, duty); }
void actuator_motor3(float duty) { motor_cmd_can2(0x102, duty); }
void actuator_motor4(float duty) { motor_cmd_can2(0x103, duty); }
void actuator_motor5(float duty) { motor_cmd_can1(0x104, duty); }

void actuator_dribbler_up() { motor_cmd_can1(0x105, 0.8); }
void actuator_dribbler_down() { motor_cmd_can1(0x105, 0); }

void actuator_kicker_cmd_voltage(float voltage)
{
  uint8_t senddata_kick[8] = {0};
  float_to_uchar4(&senddata_kick[4], voltage);

  senddata_kick[0] = 0;
  can1_send(0x110, senddata_kick);
  can2_send(0x110, senddata_kick);
}

void actuator_kicker_cmd(uint8_t id, uint8_t param)
{
  /* id 0: charge voltage [4-7]
   * id 1: 0=>standby   1=>charge
	 * id 2: 0=>straight  1=>chip
	 * id 3: kick strength 0~255
	 * */
  uint8_t senddata_kick[8] = {0};
  senddata_kick[0] = id;
  senddata_kick[1] = param;
  senddata_kick[2] = 200;
  can1_send(0x110, senddata_kick);
  can2_send(0x110, senddata_kick);
}

void kicker_select_straight(void) { actuator_kicker_cmd(2, 0); }
void kicker_select_chip(void) { actuator_kicker_cmd(2, 1); }
void kicker_charge_start(void) { actuator_kicker_cmd(1, 1); }
void kicker_chaege_stop(void) { actuator_kicker_cmd(1, 0); }
void kicker_kick_start(float kick_power)
{
  uint8_t kick_power_param = (float)kick_power * 255.0;
  actuator_kicker_cmd(3, (uint8_t)kick_power_param);
}

void actuatorPower_ONOFF(uint8_t power_on)
{
  /*id 0=>off
	 *   1=>on
	 * */
  uint8_t senddata_power[8] = {0};
  senddata_power[0] = 0;
  senddata_power[1] = power_on;
  can1_send(0x010, senddata_power);
  can2_send(0x010, senddata_power);
}

void actuator_power_param(uint8_t id, float param)
{
  /*id 1=>minVoltage
 *   2=>maxVoltage
 *   3=>maxAmpere
 *   4=>maxtemp
 *   5=>maxtemp(solenoid)
 * */

  uint8_t senddata_power_param[8] = {0};

  senddata_power_param[0] = id;
  float_to_uchar4(&senddata_power_param[1], param);

  can1_send(0x010, senddata_power_param);
  can2_send(0x010, senddata_power_param);
}

void actuator_motor_param(uint8_t param1, float m1p, uint8_t param2, float m2p, uint8_t param3, float m3p, uint8_t param4, float m4p, uint8_t param5, float m5p)
{
  uint8_t send_data_param[8] = {0};

  float_to_uchar4(&send_data_param[1], m1p);
  send_data_param[0] = param1;
  can1_send(0x300, send_data_param);

  float_to_uchar4(&send_data_param[1], m2p);
  send_data_param[0] = param2;
  can1_send(0x301, send_data_param);

  float_to_uchar4(&send_data_param[1], m3p);
  send_data_param[0] = param3;
  can2_send(0x302, send_data_param);

  float_to_uchar4(&send_data_param[1], m4p);
  send_data_param[0] = param4;
  can2_send(0x303, send_data_param);

  float_to_uchar4(&send_data_param[1], m5p);
  send_data_param[0] = param5;
  can1_send(0x304, send_data_param);
}

void actuator_buzzer(uint16_t ontime, uint16_t offtime)
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 250);
  HAL_Delay(ontime);

  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
  HAL_Delay(offtime);
}

void actuator_buzzer_on()
{
  __HAL_TIM_SET_PRESCALER(&htim5, 170);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 250);
}

void actuator_buzzer_off() { __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0); }

void actuator_buzzer_frq_on(float frq)
{
  __HAL_TIM_SET_PRESCALER(&htim5, 170 * 2000 / frq);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 250);
}
void actuator_buzzer_frq(float frq, uint16_t time)
{
  actuator_buzzer_frq_on(frq);
  HAL_Delay(time);
  actuator_buzzer_off();
}

void morse_short() { actuator_buzzer(40, 20); }
void morse_long() { actuator_buzzer(120, 20); }

void morse_machine_name()
{
  //i
  morse_short();
  morse_short();

  HAL_Delay(80);

  //b
  morse_long();
  morse_short();
  morse_short();
  morse_short();

  HAL_Delay(80);

  //i
  morse_short();
  morse_short();

  HAL_Delay(80);

  //s
  morse_short();
  morse_short();
  morse_short();

  HAL_Delay(300);

  //o
  morse_long();
  morse_long();
  morse_long();

  HAL_Delay(80);

  //r
  morse_long();
  morse_short();
  morse_long();

  HAL_Delay(80);

  //i
  morse_short();
  morse_short();

  HAL_Delay(80);

  //o
  morse_long();
  morse_long();
  morse_long();

  HAL_Delay(80);

  //n
  morse_long();
  morse_short();

  HAL_Delay(80);
}
void morse_period()
{
  morse_short();
  morse_long();
  morse_short();
  morse_long();
  morse_short();
  morse_long();
  morse_short();
  morse_long();

  HAL_Delay(80);
}

void morse_number(uint8_t value)
{
  switch (value) {
    case 0:
      morse_long();
      morse_long();
      morse_long();
      morse_long();
      morse_long();
      break;
    case 1:
      morse_short();
      morse_long();
      morse_long();
      morse_long();
      morse_long();
      break;
    case 2:
      morse_short();
      morse_short();
      morse_long();
      morse_long();
      morse_long();
      break;
    case 3:
      morse_short();
      morse_short();
      morse_short();
      morse_long();
      morse_long();
      break;
    case 4:
      morse_short();
      morse_short();
      morse_short();
      morse_short();
      morse_long();
      break;
    case 5:
      morse_short();
      morse_short();
      morse_short();
      morse_short();
      morse_short();
      break;
    case 6:
      morse_long();
      morse_short();
      morse_short();
      morse_short();
      morse_short();
      break;
    case 7:
      morse_long();
      morse_long();
      morse_short();
      morse_short();
      morse_short();
      break;
    case 8:
      morse_long();
      morse_long();
      morse_long();
      morse_short();
      morse_short();
      break;
    case 9:
      morse_long();
      morse_long();
      morse_long();
      morse_long();
      morse_short();
      break;
  }

  HAL_Delay(80);
}

void actuator_motor_calib(int board)
{
  uint8_t dummy[8] = {0};
  if (board == 0) {
    can1_send(0x310, dummy);
  } else if (board == 1) {
    can2_send(0x310, dummy);
  }
}
