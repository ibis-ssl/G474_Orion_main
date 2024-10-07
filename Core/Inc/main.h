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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void p(const char * format, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_G_Pin GPIO_PIN_13
#define LED_G_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOC
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_4
#define IMU_INT_GPIO_Port GPIOC
#define IMU_FSYNC_Pin GPIO_PIN_5
#define IMU_FSYNC_GPIO_Port GPIOC
#define V_SENSE_Pin GPIO_PIN_0
#define V_SENSE_GPIO_Port GPIOB
#define C_SENSE_Pin GPIO_PIN_1
#define C_SENSE_GPIO_Port GPIOB
#define DIP_2_Pin GPIO_PIN_10
#define DIP_2_GPIO_Port GPIOA
#define TIM5_CH2_Buzzer_Pin GPIO_PIN_12
#define TIM5_CH2_Buzzer_GPIO_Port GPIOC
#define DIP_3_Pin GPIO_PIN_2
#define DIP_3_GPIO_Port GPIOD
#define UART2_TX_ETH_Pin GPIO_PIN_3
#define UART2_TX_ETH_GPIO_Port GPIOB
#define UART2_RX_ETH_Pin GPIO_PIN_4
#define UART2_RX_ETH_GPIO_Port GPIOB
#define DIP_0_Pin GPIO_PIN_5
#define DIP_0_GPIO_Port GPIOB
#define DIP_1_Pin GPIO_PIN_6
#define DIP_1_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_7
#define LED_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
