/*
 * omni_wheel.c
 *
 *  Created on: Sep 4, 2019
 *      Author: okada_tech
 */
#include "omni_wheel.h"

/*motor place
 *
 *    	  5(dribble)
 *  motor4  		motor1
 *           ^
 *			 |
 *  motor3      	motor2
 *
 * */


const float32_t rotation_length_omni = omni_diameter*M_PI;
const float32_t gear_ratio=1.0;
const float32_t sinM1 = sin(    M_PI/6.0);
const float32_t sinM2 = sin(7.0*M_PI/4.0);
const float32_t sinM3 = sin(5.0*M_PI/4.0);
const float32_t sinM4 = sin(5.0*M_PI/6.0);
const float32_t cosM1 = cos(    M_PI/6.0);
const float32_t cosM2 = cos(7.0*M_PI/4.0);
const float32_t cosM3 = cos(5.0*M_PI/4.0);
const float32_t cosM4 = cos(5.0*M_PI/6.0);

void omni_move(float32_t vel_y_omni,float32_t vel_x_omni,float32_t omega_omni,float32_t duty_Limit){
	v_round=robot_radius*omega_omni;

	m1=((vel_x_omni*sinM1)+(vel_y_omni*cosM1)+v_round)/rotation_length_omni*gear_ratio;
	m2=((vel_x_omni*sinM2)+(vel_y_omni*cosM2)+v_round)/rotation_length_omni*gear_ratio;
	m3=((vel_x_omni*sinM3)+(vel_y_omni*cosM3)+v_round)/rotation_length_omni*gear_ratio;
	m4=((vel_x_omni*sinM4)+(vel_y_omni*cosM4)+v_round)/rotation_length_omni*gear_ratio;

	actuator_motor1(m1,duty_Limit);
	actuator_motor2(m2,duty_Limit);
	actuator_motor3(m3,duty_Limit);
	actuator_motor4(m4,duty_Limit);

	/*if(abs(m1)>10.0 || abs(m2)>10.0 ||abs(m3)>10.0 ||abs(m4)>10.0 || stall>0){
		if(cnt_motor>70 || stall>0){
			cnt_motor=0;
			if(stall>0){
				stall_move();
			}
			else if(abs(motor_feedback_velocity[0])<0.01 && abs(m1)>10.0){
				stall_move();
			}
			else if(abs(motor_feedback_velocity[1])<0.01 && abs(m2)>10.0){
				stall_move();
			}
			else if(abs(motor_feedback_velocity[2])<0.01 && abs(m3)>10.0){
				stall_move();
			}
			else if(abs(motor_feedback_velocity[3])<0.01 && abs(m4)>10.0){
				stall_move();
			}
			else{
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			}
		}
		cnt_motor++;
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

		cnt_motor=0;
		stall=0;
	}*/

}

void stall_move(){

	printf("stall !!!!!!!!!!!!!!!!!\r\n");

	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 250);

	actuator_motor1(0,0);
	actuator_motor2(0,0);
	actuator_motor3(0,0);
	actuator_motor4(0,0);
	if(stall>8){
		stall=0;
	}
	else{
		stall++;
	}
}
