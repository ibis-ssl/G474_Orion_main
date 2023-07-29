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


const float32_t rotation_length_omni = OMNI_DIAMETER*M_PI;
const float32_t sinM1 = sin(    M_PI/6.0);
const float32_t sinM2 = sin(7.0*M_PI/4.0);
const float32_t sinM3 = sin(5.0*M_PI/4.0);
const float32_t sinM4 = sin(5.0*M_PI/6.0);
const float32_t cosM1 = cos(    M_PI/6.0);
const float32_t cosM2 = cos(7.0*M_PI/4.0);
const float32_t cosM3 = cos(5.0*M_PI/4.0);
const float32_t cosM4 = cos(5.0*M_PI/6.0);

void omni_move(float32_t vel_y_omni,float32_t vel_x_omni,float32_t omega_omni,float32_t duty_Limit){
	float32_t v_round;
	float32_t m1, m2, m3, m4;

	v_round=ROBOT_RADIUS*omega_omni;

	m1=((vel_x_omni*sinM1)+(vel_y_omni*cosM1)+v_round)/rotation_length_omni;
	m2=((vel_x_omni*sinM2)+(vel_y_omni*cosM2)+v_round)/rotation_length_omni;
	m3=((vel_x_omni*sinM3)+(vel_y_omni*cosM3)+v_round)/rotation_length_omni;
	m4=((vel_x_omni*sinM4)+(vel_y_omni*cosM4)+v_round)/rotation_length_omni;

	actuator_motor1(m1,duty_Limit);
	actuator_motor2(m2,duty_Limit);
	actuator_motor3(m3,duty_Limit);
	actuator_motor4(m4,duty_Limit);
}
