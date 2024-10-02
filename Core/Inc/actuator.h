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
void actuator_motor1(float m1);
void actuator_motor2(float m2);
void actuator_motor3(float m3);
void actuator_motor4(float m4);
void actuator_motor5(float m5);
void actuator_dribbler_up();
void actuator_dribbler_down();
void actuator_motor_param(uint8_t param1, float m1p, uint8_t param2, float m2p, uint8_t param3, float m3p, uint8_t param4, float m4p, uint8_t param5, float m5p);
void actuator_kicker_straight(void);
void actuator_kicker_chip(void);
void actuator_kicker_charge_start(void);
void actuator_kicker_chaege_stop(void);
void actuator_kicker_kick(float kick_power);
void actuator_kicker_voltage(float voltage);
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
