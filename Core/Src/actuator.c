/*
 * actuator.c
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */
#include "actuator.h"

static void motor_cmd(uint16_t motor_id, float duty)
{
  uint8_t senddata_motor[8];
  float_to_uchar4(senddata_motor, duty);
  can1_send(motor_id, senddata_motor);
}

void actuator_motor1(float duty, float duty_Limit1) { motor_cmd(0x100, duty); }
void actuator_motor2(float duty, float duty_Limit2) { motor_cmd(0x101, duty); }
void actuator_motor3(float duty, float duty_Limit3) { motor_cmd(0x102, duty); }
void actuator_motor4(float duty, float duty_Limit4) { motor_cmd(0x103, duty); }
void actuator_motor5(float duty, float duty_Limit5) { motor_cmd(0x104, duty); }

void actuator_kicker_voltage(float voltage)
{
  uint8_t senddata_kick[8];
  uint8_t senddata_voltage[4];
  float_to_uchar4(senddata_voltage, voltage);

  senddata_kick[0] = 0;
  senddata_kick[1] = senddata_voltage[0];
  senddata_kick[2] = senddata_voltage[1];
  senddata_kick[3] = senddata_voltage[2];
  senddata_kick[4] = senddata_voltage[3];
  can1_send(0x110, senddata_kick);
}

void actuator_kicker(uint8_t id, uint8_t param)
{
  /* id 1: 0=>standby   1=>charge
	 * id 2: 0=>straight  1=>chip
	 * id 3: kick strength 0~255
	 * */
  uint8_t senddata_kick[8];
  senddata_kick[0] = id;
  senddata_kick[1] = param;
  can1_send(0x110, senddata_kick);
}

void actuator_power_ONOFF(uint8_t power_on)
{
  /*id 0=>off
	 *   1=>on
	 * */
  uint8_t senddata_power[8];
  senddata_power[0] = 0;
  senddata_power[1] = power_on;
  can1_send(0x010, senddata_power);
}

void actuator_system_shutdown()
{
  uint8_t senddata_shutdown[8];
  senddata_shutdown[0] = 255;
  senddata_shutdown[1] = 0xFF;
  senddata_shutdown[2] = 0x00;
  senddata_shutdown[3] = 0xFF;
  can1_send(0x010, senddata_shutdown);
}

void actuator_power_param(uint8_t id, float param)
{
  /*id 1=>minVoltage
 *   2=>maxVoltage
 *   3=>maxAmpere
 *   4=>maxtemp
 *   5=>maxtemp(solenoid)
 * */

  uint8_t senddata_power_param_temp[8];
  uint8_t senddata_power_param[8];

  float_to_uchar4(senddata_power_param_temp, param);
  senddata_power_param[0] = id;
  senddata_power_param[1] = senddata_power_param_temp[0];
  senddata_power_param[2] = senddata_power_param_temp[1];
  senddata_power_param[3] = senddata_power_param_temp[2];
  senddata_power_param[4] = senddata_power_param_temp[3];

  can1_send(0x010, senddata_power_param);
}

void actuator_motor_param(uint8_t param1, float m1p, uint8_t param2, float m2p, uint8_t param3, float m3p, uint8_t param4, float m4p, uint8_t param5, float m5p)
{
  uint8_t senddata_temp[4];
  uint8_t senddata_param[8];

  float_to_uchar4(&senddata_param[1], m1p);
  senddata_param[0] = param1;
  can1_send(0x300, senddata_param);

  float_to_uchar4(&senddata_param[1], m2p);
  senddata_param[0] = param2;
  can1_send(0x301, senddata_param);

  float_to_uchar4(&senddata_param[1], m3p);
  senddata_param[0] = param3;
  can2_send(0x302, senddata_param);

  float_to_uchar4(&senddata_param[1], m4p);
  senddata_param[0] = param4;
  can2_send(0x303, senddata_param);

  float_to_uchar4(&senddata_param[1], m5p);
  senddata_param[0] = param5;
  can1_send(0x304, senddata_param);
}

void actuator_buzzer(uint16_t ontime, uint16_t offtime)
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 250);
  HAL_Delay(ontime);

  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
  HAL_Delay(offtime);
}

void actuator_buzzer_on() { __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 250); }
void actuator_buzzer_off() { __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0); }