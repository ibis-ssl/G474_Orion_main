/*
 * omni_wheel.c
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */
#include "omni_wheel.h"

#include "actuator.h"
#include "management.h"
#include "math.h"
#include "util.h"
/*motor place
 *
 *    	  5(dribble)
 *  motor4  		motor1
 *           ^
 *			 |
 *  motor3      	motor2
 *
 * */
// 0.056 * 3.14
static float rotation_length_omni = OMNI_DIAMETER * M_PI;
static const float sinM1 = sin(30 * M_PI / 180);
static const float cosM1 = cos(30 * M_PI / 180);

static const float sinM2 = sin(315 * M_PI / 180);
static const float cosM2 = cos(315 * M_PI / 180);

static const float sinM3 = sin(225 * M_PI / 180);
static const float cosM3 = cos(225 * M_PI / 180);

static const float sinM4 = sin(150 * M_PI / 180);
static const float cosM4 = cos(150 * M_PI / 180);

void omniMove(output_t * output, float out_limit)
{
  float rotation_omega_motor;

  rotation_omega_motor = ROBOT_RADIUS * output->omega;

  output->motor_voltage[0] = ((output->velocity[1] * sinM1) + (output->velocity[0] * cosM1) + rotation_omega_motor) / rotation_length_omni;
  output->motor_voltage[1] = ((output->velocity[1] * sinM2) + (output->velocity[0] * cosM2) + rotation_omega_motor) / rotation_length_omni;
  output->motor_voltage[2] = ((output->velocity[1] * sinM3) + (output->velocity[0] * cosM3) + rotation_omega_motor) / rotation_length_omni;
  output->motor_voltage[3] = ((output->velocity[1] * sinM4) + (output->velocity[0] * cosM4) + rotation_omega_motor) / rotation_length_omni;

  omniMoveIndiv(output, out_limit);
}

void omniMoveIndiv(output_t * output, float out_limit)
{
  for (int i = 0; i < 4; i++) {
    output->motor_voltage[i] = clampSize(output->motor_voltage[i], out_limit);
  }

  actuator_motor1(output->motor_voltage[0]);
  actuator_motor2(output->motor_voltage[1]);
  actuator_motor3(output->motor_voltage[2]);
  actuator_motor4(output->motor_voltage[3]);
}

void omniStopAll(output_t * output)
{
  output->motor_voltage[0] = 0;
  output->motor_voltage[1] = 0;
  output->motor_voltage[2] = 0;
  output->motor_voltage[3] = 0;

  actuator_motor1(output->motor_voltage[0]);
  actuator_motor2(output->motor_voltage[1]);
  actuator_motor3(output->motor_voltage[2]);
  actuator_motor4(output->motor_voltage[3]);
}