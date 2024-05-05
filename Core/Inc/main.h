/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

enum {
  MAIN_MODE_COMBINATION_CONTROL = 0,
  MAIN_MODE_SPEED_CONTROL_ONLY,
  MAIN_MODE_CMD_DEBUG_MODE,
  MAIN_MODE_MOTOR_TEST,
  MAIN_MODE_DRIBBLER_TEST,
  MAIN_MODE_KICKER_AUTO_TEST,
  MAIN_MODE_KICKER_MANUAL,
  MAIN_MODE_NONE,
  MAIN_MODE_MOTOR_CALIBRATION,
  MAIN_MODE_ERROR,
};

enum {
  BLDC_ERROR_NONE = 0,
  BLDC_ERROR_UNDER_VOLTAGE = 0x0001,
  BLDC_ERROR_OVER_CURRENT = 0x0002,
  BLDC_ERROR_MOTOR_OVER_HEAT = 0x0004,
  BLDC_ERROR_OVER_LOAD = 0x0008,
  BLDC_ERROR_ENC_ERROR = 0x0010,
  BLDC_ERROR_OVER_VOLTAGE = 0x0020,
};

enum {
  BOOST_ERROR_NONE = 0,
  BOOST_ERROR_UNDER_VOLTAGE = 0x0001,
  BOOST_ERROR_OVER_VOLTAGE = 0x0002,
  BOOST_ERROR_OVER_CURRENT = 0x0004,
  BOOST_ERROR_SHORT_CURCUIT = 0x0008,
  BOOST_ERROR_CHARGE_TIME = 0x0010,
  BOOST_ERROR_CHARGE_POWER = 0x0020,
  BOOST_ERROR_DISCHARGE = 0x0040,
  BOOST_ERROR_PARAMETER = 0x0080,
  BOOST_ERROR_COMMAND = 0x0100,
  BOOST_ERROR_NO_CAP = 0x0200,
  BOOST_ERROR_DISCHARGE_FAIL = 0x0400,
  BOOST_ERROR_GD_POWER_FAIL = 0x0800,
  BOOST_ERROR_COIL_OVER_HEAT = 0x1000,
  BOOST_ERROR_FET_OVER_HEAT = 0x2000,
};

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void p(const char * format, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM6_CH1_Buzzer_Pin GPIO_PIN_0
#define TIM6_CH1_Buzzer_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define V_SENSE_Pin GPIO_PIN_0
#define V_SENSE_GPIO_Port GPIOB
#define C_SENSE_Pin GPIO_PIN_1
#define C_SENSE_GPIO_Port GPIOB
#define UART2_TX_ETH_Pin GPIO_PIN_3
#define UART2_TX_ETH_GPIO_Port GPIOB
#define UART2_RX_ETH_Pin GPIO_PIN_4
#define UART2_RX_ETH_GPIO_Port GPIOB
#define CS_CM4_Pin GPIO_PIN_9
#define CS_CM4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
