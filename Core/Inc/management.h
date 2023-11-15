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
#include "icm20602_spi.h"
#include "actuator.h"
#include "myatan2.h"
#include "omni_wheel.h"
#include "can_ibis.h"
#include "util.h"

extern float32_t motor_voltage[4];

#define CAN_RX_DATA_SIZE 8
#define CAN_TX_DATA_SIZE 8
#define OMNI_DIAMETER 0.056
#define ROBOT_RADIUS  0.080
#define RX_BUF_SIZE_ETHER 32
#define TX_BUF_SIZE_ETHER 32



//extern float32_t voltage[6];

#endif /* MANAGEMENT_H_ */
