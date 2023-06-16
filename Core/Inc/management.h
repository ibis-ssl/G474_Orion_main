/*
 * managiment.h
 *
 *  Created on: Sep 1, 2019
 *      Author: okada_tech
 */

#ifndef MANAGEMENT_H_
#define MANAGEMENT_H_

#include "stm32g4xx_hal.h"
#include "adc.h"
#include "fdcan.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include "math.h"
#include "stdio.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#define ARM_MATH_CM4
#include "arm_math.h"
#include "microsectimer.h"
#include "xprintf.h"
#include "icm20602_spi.h"
#include "actuator.h"
#include "myatan2.h"
#include "omni_wheel.h"
#include "can_ibis.h"
#include "util.h"



extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern FDCAN_FilterTypeDef  sFilterConfig;

extern TIM_MasterConfigTypeDef sMasterConfig;
extern TIM_OC_InitTypeDef sConfigOC;
extern TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

#define can_RX_data 8
#define can_TX_data 8
#define omni_diameter 0.056
#define robot_radius  0.080
#define Rxbufsize_from_Ether 14


extern float pitchAngle;
extern float rollAngle;
extern float yawAngle;
extern float pitchAngle_rad;
extern float rollAngle_rad;
extern float yawAngle_rad;
extern float acc[3];
extern float gyro[3];
extern float acc_comp[3];
extern float gyro_comp[3];
extern float IMU_tmp;
extern char orientation;
extern float gyro_prv[3];
extern float acc_prv[3];
extern float acc_off[3];
extern float gyro_off[3];

extern uint8_t data_from_ether[Rxbufsize_from_Ether-1];
extern uint8_t TX_data_UART[9];
extern uint16_t Csense[1];
extern uint16_t Vsense[1];
extern uint16_t SWdata[1];
extern uint8_t Emargency;
extern uint8_t TxData[can_TX_data];
extern uint8_t RxData[can_RX_data];
extern uint32_t TxMailbox;
extern float32_t motor_feedback[5];
extern float32_t motor_feedback_velocity[5];
extern float32_t voltage[6];
extern float32_t tempercher[6];
extern float32_t amplitude[5];
extern float32_t power_amp;
extern float32_t vel_surge;
extern float32_t vel_sway;
extern float32_t omega;
extern float32_t drible_power;
extern float32_t kick_power;
extern float32_t theta_vision;
extern float32_t theta_target;
extern uint8_t chipEN;
extern uint8_t ball[4];
extern uint16_t mouse[2];
extern uint8_t error_No[4];
extern float32_t m1,m2,m3,m4;
extern float32_t v_round;
extern float32_t ball_x,ball_y,robot_x,robot_y,robot_x_target,robot_y_target;
extern uint8_t keeper_EN,stall;
extern uint16_t cnt_motor;
extern uint8_t check_motor1,check_motor2,check_motor3,check_motor4,check_power,check_FC;

#endif /* MANAGEMENT_H_ */
