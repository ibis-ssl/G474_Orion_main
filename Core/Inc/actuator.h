/*
 * actuator.h
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include <stdbool.h>
#include <string.h>

#include "management.h"
void actuator_motor_fr(float m1);
void actuator_motor_br(float m2);
void actuator_motor_bl(float m3);
void actuator_motor_fl(float m4);
void actuator_motor_drv(float m5);
void actuator_motor_param(uint8_t param1, float m1p, uint8_t param2, float m2p, uint8_t param3, float m3p, uint8_t param4, float m4p, uint8_t param5, float m5p);
void kicker_select_straight(void);
void kicker_select_chip(void);
void kicker_charge_start(void);
void kicker_charge_stop(void);
void kicker_kick_start(float kick_power);
void actuator_kicker_cmd_voltage(float voltage);
void actuatorPower_ONOFF(uint8_t power_on);
void actuator_power_param(uint8_t id, float param);
void actuator_buzzer(uint16_t ontime, uint16_t offtime);
void actuator_buzzer_on();
void actuator_buzzer_off();
void actuator_buzzer_frq_on(float frq);
void actuator_buzzer_frq(float frq, uint16_t time);

void morse_machine_name();
void morse_short();
void morse_long();
void morse_period();
void morse_number(uint8_t value);

void actuator_motor_calib(int board);

#endif /* ACTUATOR_H_ */
