/*
 * omni_wheel.c
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */
#include "omni_wheel.h"

#include "actuator.h"
#include "management.h"

/*motor place
 *
 *    	  5(dribble)
 *  motor4  		motor1
 *           ^
 *			 |
 *  motor3      	motor2
 *
 * */

float rotation_length_omni = OMNI_DIAMETER * M_PI;
const float sinM1 = sin(30 * M_PI / 180);
const float cosM1 = cos(30 * M_PI / 180);

const float sinM2 = sin(315 * M_PI / 180);
const float cosM2 = cos(315 * M_PI / 180);

const float sinM3 = sin(225 * M_PI / 180);
const float cosM3 = cos(225 * M_PI / 180);

const float sinM4 = sin(150 * M_PI / 180);
const float cosM4 = cos(150 * M_PI / 180);

void omni_move(float vel_y_robot, float vel_x_robot, float omega_roboot, float duty_limit, output_t * output)
{
  float rotation_omega_motor;

  rotation_omega_motor = ROBOT_RADIUS * omega_roboot;

  output->motor_voltage[0] = ((vel_x_robot * sinM1) + (vel_y_robot * cosM1) + rotation_omega_motor) / rotation_length_omni;
  output->motor_voltage[1] = ((vel_x_robot * sinM2) + (vel_y_robot * cosM2) + rotation_omega_motor) / rotation_length_omni;
  output->motor_voltage[2] = ((vel_x_robot * sinM3) + (vel_y_robot * cosM3) + rotation_omega_motor) / rotation_length_omni;
  output->motor_voltage[3] = ((vel_x_robot * sinM4) + (vel_y_robot * cosM4) + rotation_omega_motor) / rotation_length_omni;

  actuator_motor1(output->motor_voltage[0], duty_limit);
  actuator_motor2(output->motor_voltage[1], duty_limit);
  actuator_motor3(output->motor_voltage[2], duty_limit);
  actuator_motor4(output->motor_voltage[3], duty_limit);
}
