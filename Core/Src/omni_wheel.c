/*
 * omni_wheel.c
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */
#include "omni_wheel.h"

#include "arm_math.h"

/*motor place
 *
 *    	  5(dribble)
 *  motor4  		motor1
 *           ^
 *			 |
 *  motor3      	motor2
 *
 * */

const float32_t rotation_length_omni = OMNI_DIAMETER * M_PI;
const float32_t sinM1 = sin(M_PI / 6.0);
const float32_t sinM2 = sin(7.0 * M_PI / 4.0);
const float32_t sinM3 = sin(5.0 * M_PI / 4.0);
const float32_t sinM4 = sin(5.0 * M_PI / 6.0);
const float32_t cosM1 = cos(M_PI / 6.0);
const float32_t cosM2 = cos(7.0 * M_PI / 4.0);
const float32_t cosM3 = cos(5.0 * M_PI / 4.0);
const float32_t cosM4 = cos(5.0 * M_PI / 6.0);

void omni_move(float32_t vel_y_robot, float32_t vel_x_robot, float32_t omega_roboot, float32_t duty_limit)
{
  float32_t rotation_omega_motor;

  rotation_omega_motor = ROBOT_RADIUS * omega_roboot;

  output.motor_voltage[0] = ((vel_x_robot * sinM1) + (vel_y_robot * cosM1) + rotation_omega_motor) / rotation_length_omni;
  output.motor_voltage[1] = ((vel_x_robot * sinM2) + (vel_y_robot * cosM2) + rotation_omega_motor) / rotation_length_omni;
  output.motor_voltage[2] = ((vel_x_robot * sinM3) + (vel_y_robot * cosM3) + rotation_omega_motor) / rotation_length_omni;
  output.motor_voltage[3] = ((vel_x_robot * sinM4) + (vel_y_robot * cosM4) + rotation_omega_motor) / rotation_length_omni;

  actuator_motor1(output.motor_voltage[0], duty_limit);
  actuator_motor2(output.motor_voltage[1], duty_limit);
  actuator_motor3(output.motor_voltage[2], duty_limit);
  actuator_motor4(output.motor_voltage[3], duty_limit);
}
