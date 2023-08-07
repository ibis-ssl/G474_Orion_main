/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dma_printf.h"
#include "dma_scanf.h"
#include "management.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE * f)
#endif /* __GNUC__ */

void __io_putchar(uint8_t ch) { HAL_UART_Transmit(&hlpuart1, &ch, 1, 1); }

/*int __io_putchar(int ch)
{
	  dma_printf_putc(ch&0xFF);
		return ch;
}

int __io_getchar(void)
{
	  return dma_scanf_getc_blocking();
}*/

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rxbuf_from_ether[RX_BUF_SIZE_ETHER];
uint8_t sw_mode;
uint8_t ether_connect = 0;
uint32_t connection_check_cnt = 0;

const float32_t OMNI_DIR_LENGTH = OMNI_DIAMETER * M_PI;

#define OMNI_OUTPUT_LIMIT (4)
#define OMNI_OUTPUT_GAIN (-0.5)
const float32_t OMNI_ROTATION_LENGTH = (0.07575);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs);
void maintask_run();
void maintask_stop();
void maintask_emargency();
void maintask_state_stop();
long map(long x, long in_min, long in_max, long out_min, long out_max);
float getAngleDiff(float angle_rad1, float angle_rad2);
float normalizeAngle(float angle_rad);
uint32_t HAL_GetTick(void) { return uwTick; }
uint8_t decode_SW(uint16_t sw_raw_data);

float pitch_angle;
float roll_angle;
float yaw_angle, pre_yaw_angle;
float yaw_angle_rad, pre_yaw_angle_rad;

volatile int kick_state, kick_time;
uint8_t data_from_ether[RX_BUF_SIZE_ETHER];
uint8_t tx_data_uart[9];

uint16_t Csense[1];
uint16_t Vsense[1];
uint16_t SWdata[1];
uint8_t emergency_flag;
float32_t motor_feedback[5];
float32_t motor_feedback_velocity[5];
float32_t voltage[6];
float32_t power_voltage[6];
float32_t temperature[6];
float32_t amplitude[5];
uint8_t ball_detection[4];

uint8_t error_no[4];

struct
{
  float target_theta, global_vision_theta;
  float drible_power;
  float kick_power;
  uint8_t keeper_en;
  uint8_t chip_en;
  float local_target_speed[2];
  int global_robot_position[2];
  int global_global_target_position[2];
  int global_ball_position[2];
  uint8_t allow_local_feedback;
} ai_cmd;

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_FilterTypeDef sFilterConfig;

TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
UART_HandleTypeDef * huart_xprintf;

int16_t mouse_raw_latest[2];
float mouse_odom[2];

float32_t motor_enc_angle[5];
float32_t pre_motor_enc_angle[5];
float32_t omni_angle_diff[4];

float omni_travel[2];
float32_t floor_odom_diff[2] = {0, 0}, robot_pos_diff[2] = {0, 0};
float32_t tar_pos[2] = {0, 0};
float32_t tar_vel[2] = {0, 0};  // m/s

float32_t omni_odom[2] = {0};
float32_t pre_omni_odom[2] = {0};
float32_t omni_odom_speed[2] = {0};
#define SPEED_LOG_BUF_SIZE 100
float32_t omni_odom_speed_log[2][SPEED_LOG_BUF_SIZE] = {0};  // 2ms * 100cycle = 200ms

char Tx_printf_data[100];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_ADC5_Init();
  MX_ADC3_Init();
  MX_FDCAN2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  kick_state = 0;
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  for (int i = 0; i < 3; i++) {
    actuator_buzzer(20, 20);
  }

  HAL_UART_Init(&hlpuart1);
  setbuf(stdin, NULL);
  setbuf(stdout, NULL);
  setbuf(stderr, NULL);
  dma_printf_init(&hlpuart1);
  dma_scanf_init(&hlpuart1);

  printf("start\r\n");
  can1_init_ibis(&hfdcan1);
  can2_init_ibis(&hfdcan2);

  HAL_FDCAN_Start(&hfdcan1);
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    Error_Handler();
  }

  HAL_FDCAN_Start(&hfdcan2);
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    Error_Handler();
  }

  HAL_UART_Init(&huart2);
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)rxbuf_from_ether, RX_BUF_SIZE_ETHER);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Vsense, 1);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t *)Csense, 1);
  HAL_ADC_Start_DMA(&hadc5, (uint32_t *)SWdata, 1);

  actuator_power_ONOFF(0);
  HAL_Delay(20);

  actuator_motor1(0.0, 0.0);
  actuator_motor2(0.0, 0.0);
  actuator_motor3(0.0, 0.0);
  actuator_motor4(0.0, 0.0);
  actuator_motor5(0.0, 0.0);

  actuator_kicker(1, 1);
  actuator_kicker_voltage(250.0);
  actuator_power_param(1, 15.0);  // min voltage
  actuator_power_param(2, 35.0);  // max voltage
  actuator_power_param(3, 50.0);  // max current
  actuator_power_param(4, 90.0);  // max temp(fet)
  actuator_power_param(5, 90.0);  // max temp(solenoid)

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
  ICM20602_init();
  ICM20602_IMU_calibration2();
  ICM20602_clearAngle();

  // uint8_t senddata_calib[8];
  // can1_send(0x340, senddata_calib);
  // can2_send(0x340, senddata_calib);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
  actuator_power_ONOFF(1);

  for (int i = 0; i < 3; i++) {
    actuator_buzzer(40, 40);
  }

  HAL_Delay(100);
  HAL_TIM_Base_Start_IT(&htim7);

  mouse_odom[0] = 0;
  mouse_odom[1] = 0;
  omni_odom[0] = 0;
  omni_odom[1] = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  static uint8_t pre_sw_mode;
  ICM20602_read_IMU_data(0.002);
  pre_sw_mode = sw_mode;
  sw_mode = 15 - (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) + (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) << 1) + (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) << 3) + (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) << 2));

  if (sw_mode != pre_sw_mode) {
    omni_odom[0] = tar_pos[1];
    omni_odom[1] = tar_pos[1];
    ai_cmd.local_target_speed[0] = 0;
    ai_cmd.local_target_speed[1] = 0;
  }
  switch (sw_mode) {
    case 0:  // main without debug
      yaw_angle = yaw_angle - (getAngleDiff(yaw_angle * PI / 180.0, ai_cmd.global_vision_theta) * 180.0 / PI) * 0.001;
      maintask_run();
      break;

    case 1:  // main debug
      yaw_angle = yaw_angle - (getAngleDiff(yaw_angle * PI / 180.0, ai_cmd.global_vision_theta) * 180.0 / PI) * 0.001;
      maintask_run();
      break;

    case 2:  // calibration motor
      if (decode_SW(SWdata[0]) & 0b00010000) {
        uint8_t senddata_calib[8];
        can1_send(0x310, senddata_calib);  // calibration
        can2_send(0x310, senddata_calib);  // calibration
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
      } else {
        omni_move(0.0, 0.0, 0.0, 0.0);
        actuator_motor5(0.0, 0.0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
      }
      break;

    case 3:  // motor test
      if (decode_SW(SWdata[0]) & 0b00000001) {
        omni_move(1.0, 0.0, 0.0, 1.0);  // fwd
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
      } else if (decode_SW(SWdata[0]) & 0b00000010) {
        omni_move(-1.0, 0.0, 0.0, 1.0);  // back
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
      } else if (decode_SW(SWdata[0]) & 0b00000100) {
        omni_move(0.0, -1.0, 0.0, 1.0);  // left
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
      } else if (decode_SW(SWdata[0]) & 0b00001000) {
        omni_move(0.0, 1.0, 0.0, 1.0);  // right
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
      } else if (decode_SW(SWdata[0]) & 0b00010000) {
        omni_move(0.0, 0.0, 7.0, 1.0);  // spin
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
      } else {
        omni_move(0.0, 0.0, 0.0, 0.0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
      }
      actuator_motor5(0.0, 0.0);
      break;

    case 4:  // drible test
      if (decode_SW(SWdata[0]) & 0b00010000) {
        actuator_motor5(0.5, 1.0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
      } else {
        actuator_motor5(0.0, 0.0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
      }
      omni_move(0.0, 0.0, 0.0, 0.0);
      break;

    case 5:
      if (decode_SW(SWdata[0]) & 0b00010000) {
        actuator_motor5(0.5, 1.0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
        if (ball_detection[0] == 1) {
          if (kick_state == 0) {
            actuator_kicker(3, 100);
            kick_state = 1;
          }
        }
        if (kick_state == 1) {
          kick_time++;
          if (kick_time > 100) {
            kick_state = 0;
            kick_time = 0;
          }
        }
      } else {
        actuator_motor5(0.0, 0.0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
        actuator_kicker(1, 1);
        actuator_kicker(2, 0);
        actuator_kicker_voltage(250.0);
        kick_state = 0;
        kick_time = 0;
      }
      omni_move(0.0, 0.0, 0.0, 0.0);
      break;

    case 6:
      if (decode_SW(SWdata[0]) & 0b00010000) {
        actuator_motor5(0.5, 1.0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
        if (ball_detection[0] == 1) {
          if (kick_state == 0) {
            actuator_kicker(3, 100);
            kick_state = 1;
          }
        }
        if (kick_state == 1) {
          kick_time++;
          if (kick_time > 100) {
            kick_state = 0;
            kick_time = 0;
          }
        }
      } else {
        actuator_motor5(0.0, 0.0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
        actuator_kicker(1, 1);
        actuator_kicker(2, 1);
        actuator_kicker_voltage(0.0);
        kick_state = 0;
        kick_time = 0;
      }
      omni_move(0.0, 0.0, 0.0, 0.0);
      break;

    default:
      maintask_stop();
      break;
  }

  /*
  static bool buzzer_state = false;
  static uint32_t buzzer_cnt = 0;
  buzzer_cnt++;
  if (buzzer_cnt > 100) {
    buzzer_cnt = 0;

    if (power_voltage[0] < 21.0) {
      if (buzzer_state == false) {
        buzzer_state = true;
        actuator_buzzer_on();
      } else {
        buzzer_state = false;
        actuator_buzzer_off();
      }
    } else {
      buzzer_state = false;
      actuator_buzzer_off();
    }
  }
*/
  static uint16_t cnt_time_100Hz;
  cnt_time_100Hz++;
  if (cnt_time_100Hz > 10) {
    cnt_time_100Hz = 0;


	sprintf(Tx_printf_data,"yaw=%+6.1f ", yaw_angle);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," motor0=%.3f motor1=%.3f motor2=%.3f motor3=%.3f",motor_feedback[0],motor_feedback[1],motor_feedback[2],motor_feedback[3]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," v0=%.3f v1=%.3f v2=%.3f v3=%.3f",voltage[0],voltage[1],voltage[2],voltage[3]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," t0=%.3f t1=%.3f t2=%.3f t3=%.3f",temperature[0],temperature[1],temperature[2],temperature[3]);
	sprintf(Tx_printf_data + strlen(Tx_printf_data)," Batt=%3.1f ", power_voltage[0]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," mouse_raw_latest:x=%+3d, y=%+3d",mouse_raw_latest[0],mouse_raw_latest[1]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," ENC %+4.1f %+4.1f %+4.1f %+4.1f ", motor_enc_angle[0], motor_enc_angle[1], motor_enc_angle[2], motor_enc_angle[3]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," con %3d ", connection_check_cnt);
	sprintf(Tx_printf_data + strlen(Tx_printf_data)," vel X %+4.1f Y %+4.1f tharW %+4.1f ", ai_cmd.local_target_speed[0], ai_cmd.local_target_speed[1], ai_cmd.target_theta);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," grbl robot X %+5d Y %+5d W %+4.1f ", ai_cmd.global_robot_position[0], ai_cmd.global_robot_position[1], ai_cmd.global_vision_theta);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," ball X %+5d Y %+5d ", ai_cmd.global_ball_position[0], ai_cmd.global_ball_position[1]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," tar X %+5d Y %+5d ", ai_cmd.global_global_target_position[0], ai_cmd.global_global_target_position[1]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," PD %+5.2f  %+5.2f ", robot_pos_diff[0], robot_pos_diff[1]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," omni X%+8.3f Y%+8.3f ", omni_odom[0], omni_odom[1]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," mouse_raw_latest X%+8.3f Y%+8.3f ", mouse_odom[0], mouse_odom[1]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," local tar X%+8.3f Y%+8.3f ", tar_pos[0], tar_pos[1]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," tarVel X%+4.3f Y%+4.3f ", tar_vel[0], tar_vel[1]);
	//sprintf(Tx_printf_data + strlen(Tx_printf_data)," );
	sprintf(Tx_printf_data + strlen(Tx_printf_data),"\r\n");

    HAL_UART_Transmit_DMA(&hlpuart1,(uint8_t *) Tx_printf_data, 100);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);

    actuator_power_ONOFF(1);
  }

  connection_check_cnt++;
  if (connection_check_cnt > 200) {
    ether_connect = 0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

  } else {
    ether_connect = 1;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (HAL_GetTick() > 2000) {
    uint8_t cnt = 0;
    while (cnt < 100) {
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 1) {
        cnt++;
        delayUs(1);
      } else {
        break;
      }
    }

    if (cnt >= 100) {
      emergency_flag = 1;
      printf("Emergency Stop !!!!!!!!!!!!!");
      for (int i = 0; i < 50; i++) {
        maintask_emargency();
      }
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
      NVIC_SystemReset();
      emergency_flag = 0;
    } else {
      emergency_flag = 0;
    }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs)
{
  uint8_t RxData[CAN_RX_DATA_SIZE];
  FDCAN_RxHeaderTypeDef RxHeader;
  uint16_t rx_can_id;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
      Error_Handler();
    }
    rx_can_id = RxHeader.Identifier;
    switch (rx_can_id) {
      // error
      case 0x000:
        error_no[0] = RxData[0];
        error_no[1] = RxData[1];
        Error_Handler();
        break;
      case 0x001:
        error_no[0] = RxData[0];
        error_no[1] = RxData[1];
        maintask_stop();
        break;
      case 0x002:
        break;
      case 0x003:
        break;
      case 0x004:
        break;

      // motor_feedback
      case 0x200:
      case 0x201:
      case 0x202:
      case 0x203:
        motor_enc_angle[rx_can_id - 0x200] = uchar4_to_float(&RxData[4]);
        motor_feedback[rx_can_id - 0x200] = uchar4_to_float(RxData);
        motor_feedback_velocity[rx_can_id - 0x200] = motor_feedback[3] * OMNI_DIR_LENGTH;
        break;

        // power_Voltage
      case 0x210:
      case 0x211:
      case 0x212:
      case 0x213:
      case 0x214:
      case 0x215:
      case 0x216:
        power_voltage[rx_can_id - 0x210] = uchar4_to_float(RxData);
        break;

      // temperature
      case 0x220:
      case 0x221:
      case 0x222:
      case 0x223:
      case 0x224:
      case 0x225:
        temperature[rx_can_id - 0x220] = uchar4_to_float(RxData);
        break;

      // amplitude
      case 0x230:
      case 0x231:
      case 0x232:
      case 0x233:
      case 0x234:
        amplitude[rx_can_id - 0x230] = uchar4_to_float(RxData);
        break;

      // ball_detection
      case 0x240:
        ball_detection[0] = RxData[0];
        ball_detection[1] = RxData[1];
        ball_detection[2] = RxData[2];
        ball_detection[3] = RxData[3];
        break;

      // mouseXY
      case 0x241:
        mouse_raw_latest[0] = (int16_t)((RxData[1] << 8) | RxData[0]);
        mouse_raw_latest[1] = (int16_t)((RxData[3] << 8) | RxData[2]);
        mouse_odom[0] += (float)mouse_raw_latest[0] / 1000;
        mouse_odom[1] += (float)mouse_raw_latest[1] / 1000;
        break;
    }
  }
}

float omega;
void theta_control(/*float global_target_thera,float robot_gyro_theta*/){
  // PID
  omega = (getAngleDiff(ai_cmd.target_theta, yaw_angle_rad) * 160.0) - (getAngleDiff(yaw_angle_rad, pre_yaw_angle_rad) * 4000.0);
  //float32_t omega = (getAngleDiff(ai_cmd.target_theta, yaw_angle_rad) * 16.0) - (getAngleDiff(yaw_angle_rad, pre_yaw_angle_rad) * 400.0);

  if (omega > 6 * M_PI) {
    omega = 6 * M_PI;
  }
  if (omega < -6 * M_PI) {
    omega = -6 * M_PI;
  }
  // omega = 0;

  // out : omega
}

void mouse_odometory()
{
  static float pre_mouse_odom[2] = {0, 0};
  float mouse_vel[2];
  mouse_vel[0] = mouse_odom[0] - pre_mouse_odom[0];
  mouse_vel[1] = mouse_odom[1] - pre_mouse_odom[1];

  pre_mouse_odom[0] = mouse_odom[0];
  pre_mouse_odom[1] = mouse_odom[1];
}

void omni_odometory(/*float motor_angle[4],float yaw_rad*/)
{
  // motor_enc_angle,/yaw_angle_rad

  for (int i = 0; i < 4; i++) {
    omni_angle_diff[i] = getAngleDiff(motor_enc_angle[i], pre_motor_enc_angle[i]);
    pre_motor_enc_angle[i] = motor_enc_angle[i];
  }

  float32_t robot_rotation_adj;
  robot_rotation_adj = normalizeAngle(yaw_angle_rad - pre_yaw_angle_rad) * OMNI_ROTATION_LENGTH;  // mm

  omni_travel[0] = omni_angle_diff[1] * OMNI_DIAMETER + robot_rotation_adj * 2;
  omni_travel[1] = omni_angle_diff[2] * OMNI_DIAMETER + robot_rotation_adj * 2;
  // spin_adjusted_result += (omni_travel[0] + omni_travel[1]) / 2;

  pre_omni_odom[0] = omni_odom[0];
  pre_omni_odom[1] = omni_odom[1];
  // pre_yaw_angle_rad
  omni_odom[0] += (omni_travel[0] * cos(yaw_angle_rad + M_PI * 3 / 4) - omni_travel[1] * cos(yaw_angle_rad + M_PI * 5 / 4)) / 2;
  omni_odom[1] += (omni_travel[0] * sin(yaw_angle_rad + M_PI * 3 / 4) - omni_travel[1] * sin(yaw_angle_rad + M_PI * 5 / 4)) / 2;

  omni_odom_speed[0] = (omni_odom[0] - pre_omni_odom[0]);
  omni_odom_speed[1] = (omni_odom[1] - pre_omni_odom[1]);

  static uint32_t odom_speed_index = 0;
  odom_speed_index++;
  if (odom_speed_index > SPEED_LOG_BUF_SIZE) {
    odom_speed_index = 0;
  }
  omni_odom_speed_log[0][odom_speed_index] = omni_odom_speed[0];
  omni_odom_speed_log[1][odom_speed_index] = omni_odom_speed[1];
}

float output_vel_surge, output_vel_sway;

void speed_control(/*float global_target_position[2],float global_robot_odom_position[2],float robot_theta*/)
{
  tar_pos[0] += tar_vel[0] / 500;
  tar_pos[1] += tar_vel[1] / 500;

  // 絶対座標系
  floor_odom_diff[0] = omni_odom[0] - tar_pos[0];
  floor_odom_diff[1] = omni_odom[1] - tar_pos[1];

  // ロボット座標系
  // X
  robot_pos_diff[0] = floor_odom_diff[0] * cos(yaw_angle_rad) + floor_odom_diff[1] * sin(yaw_angle_rad);
  // Y
  robot_pos_diff[1] = floor_odom_diff[0] * sin(yaw_angle_rad) + floor_odom_diff[1] * cos(yaw_angle_rad);
  output_vel_surge = robot_pos_diff[0] * OMNI_OUTPUT_GAIN * 500;
  output_vel_sway = robot_pos_diff[1] * OMNI_OUTPUT_GAIN * 500;
  //+target_move_speed * 2;

  float limit_gain = 0;
  if (output_vel_surge > OMNI_OUTPUT_LIMIT) {
    limit_gain = output_vel_surge / OMNI_OUTPUT_LIMIT;
    output_vel_surge = OMNI_OUTPUT_LIMIT;
    output_vel_sway /= limit_gain;
  } else if (output_vel_surge < -OMNI_OUTPUT_LIMIT) {
    limit_gain = -output_vel_surge / OMNI_OUTPUT_LIMIT;
    output_vel_surge = -OMNI_OUTPUT_LIMIT;
    output_vel_sway /= limit_gain;
  }

  if (output_vel_sway > OMNI_OUTPUT_LIMIT) {
    limit_gain = output_vel_sway / OMNI_OUTPUT_LIMIT;
    output_vel_sway = OMNI_OUTPUT_LIMIT;
    output_vel_surge /= limit_gain;
  } else if (output_vel_sway < -OMNI_OUTPUT_LIMIT) {
    limit_gain = -output_vel_sway / OMNI_OUTPUT_LIMIT;
    output_vel_sway = -OMNI_OUTPUT_LIMIT;
    output_vel_surge /= limit_gain;
  }

  // output vel X/Y
}

void maintask_run()
{
  // theta_target=0.0;
  yaw_angle_rad = yaw_angle * M_PI / 180;

  //omni_odometory();

  // 1000Hz, m/s -> m / cycle
  //tar_vel[0] = 0.05;
  tar_vel[0] = ai_cmd.local_target_speed[0];
  tar_vel[1] = ai_cmd.local_target_speed[1];

  speed_control();
  theta_control();

  pre_yaw_angle_rad = yaw_angle_rad;
  pre_yaw_angle = yaw_angle;

  if (!ether_connect && sw_mode == 0) {
    ai_cmd.local_target_speed[0] = 0;
    ai_cmd.local_target_speed[1] = 0;
    maintask_state_stop();
    return;  // without move
  }
  omni_move(output_vel_surge, output_vel_sway, omega, 1.0);

  if (ai_cmd.kick_power > 0) {
    if (ball_detection[0] == 1) {
      if (kick_state == 0) {
        uint8_t kick_power_param = (float)ai_cmd.kick_power * 255.0;
        printf(" kick=%d\r\n", kick_power_param);
        actuator_kicker(3, (uint8_t)kick_power_param);
        kick_state = 1;
      }
    }
    if (kick_state == 1) {
      kick_time++;
      if (kick_time > 100) {
        kick_state = 0;
        kick_time = 0;
      }
    }
  }

  if (ai_cmd.chip_en == 1) {
    actuator_kicker(2, 1);
  } else {
    actuator_kicker(2, 0);
  }
  actuator_kicker(1, 1);
  actuator_kicker_voltage(250.0);

  actuator_motor5(ai_cmd.drible_power, 1.0);

  uint8_t yaw_angle_send_low = ((int)yaw_angle + 360) & 0x00FF;
  uint8_t yaw_angle_send_high = (((int)yaw_angle + 360) & 0xFF00) >> 8;

  tx_data_uart[0] = 254;
  tx_data_uart[1] = (uint8_t)yaw_angle_send_low;
  tx_data_uart[2] = (uint8_t)yaw_angle_send_high;
  tx_data_uart[3] = ball_detection[0];
  tx_data_uart[4] = ball_detection[1];
  tx_data_uart[5] = ai_cmd.chip_en;
  tx_data_uart[6] = kick_state;
  tx_data_uart[7] = (uint8_t)power_voltage[4];
  HAL_UART_Transmit_DMA(&huart2, tx_data_uart, 8);
}

void maintask_emargency()
{
  actuator_motor1(0.0, 0.0);
  actuator_motor2(0.0, 0.0);
  actuator_motor3(0.0, 0.0);
  actuator_motor4(0.0, 0.0);
  actuator_motor5(0.0, 0.0);

  tx_data_uart[0] = 254;
  tx_data_uart[1] = error_no[0];
  tx_data_uart[2] = error_no[1];
  tx_data_uart[3] = error_no[2];
  tx_data_uart[4] = error_no[3];
  tx_data_uart[5] = 252;
  tx_data_uart[6] = 122;
  tx_data_uart[7] = 200;
  HAL_UART_Transmit_DMA(&huart2, tx_data_uart, 8);

  actuator_buzzer(150, 150);

  uint8_t senddata_error[8];

  can1_send(0x000, senddata_error);
  can2_send(0x000, senddata_error);

  actuator_kicker(1, 0);
  actuator_kicker_voltage(0.0);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
}

void maintask_state_stop()
{
  uint8_t yaw_angle_send_low = ((int)yaw_angle + 360) & 0x00FF;
  uint8_t yaw_angle_send_high = (((int)yaw_angle + 360) & 0xFF00) >> 8;

  omni_move(0.0, 0.0, 0.0, 0.0);
  actuator_motor5(0.0, 0.0);

  tx_data_uart[0] = 254;
  tx_data_uart[1] = (uint8_t)yaw_angle_send_low;
  tx_data_uart[2] = (uint8_t)yaw_angle_send_high;
  tx_data_uart[3] = error_no[0];
  tx_data_uart[4] = error_no[1];
  tx_data_uart[5] = 1;
  tx_data_uart[6] = 1;
  tx_data_uart[7] = (uint8_t)power_voltage[4];
  HAL_UART_Transmit_DMA(&huart2, tx_data_uart, 8);

  actuator_kicker(1, 0);
  actuator_kicker_voltage(0.0);
}

void maintask_stop()
{
  omni_move(0.0, 0.0, 0.0, 0.0);
  actuator_motor5(0.0, 0.0);

  uint8_t yaw_angle_send_low = ((int)yaw_angle + 360) & 0x00FF;
  uint8_t yaw_angle_send_high = (((int)yaw_angle + 360) & 0xFF00) >> 8;

  omni_move(0.0, 0.0, 0.0, 0.0);
  actuator_motor5(0.0, 0.0);

  tx_data_uart[0] = 254;
  tx_data_uart[1] = (uint8_t)yaw_angle_send_low;
  tx_data_uart[2] = (uint8_t)yaw_angle_send_high;
  tx_data_uart[3] = error_no[0];
  tx_data_uart[4] = error_no[1];
  tx_data_uart[5] = 0;
  tx_data_uart[6] = 0;
  tx_data_uart[7] = (uint8_t)power_voltage[4];
  HAL_UART_Transmit_DMA(&huart2, tx_data_uart, 8);

  actuator_kicker(1, 0);
  actuator_kicker_voltage(0.0);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  uint8_t start_byte_idx = 0;

  if (huart->Instance == huart2.Instance) {
    while (rxbuf_from_ether[start_byte_idx] != 254 && start_byte_idx < sizeof(rxbuf_from_ether)) {
      start_byte_idx++;
    }
    if (start_byte_idx >= sizeof(rxbuf_from_ether)) {
      for (uint8_t k = 0; k < (sizeof(data_from_ether)); k++) {
        data_from_ether[k] = 0;
      }
      return;
      //受信なしデータクリア
    } else {
      for (int i = 0; i < RX_BUF_SIZE_ETHER - 1; i++) {
        data_from_ether[i] = rxbuf_from_ether[i + 1];
      }
    }
    /*
    ai_cmd.local_target_speed[0] = ((float32_t)(data_from_ether[0] << 8 | data_from_ether[1]) - 32767.0) / 32767.0 * 7.0;
    ai_cmd.local_target_speed[1] = ((float32_t)(data_from_ether[2] << 8 | data_from_ether[3]) - 32767.0) / 32767.0 * 7.0;
    ai_cmd.global_vision_theta = ((float32_t)(data_from_ether[4] << 8 | data_from_ether[5]) - 32767) / 32767.0 * M_PI;
    ai_cmd.target_theta = ((float32_t)(data_from_ether[6] << 8 | data_from_ether[7]) - 32767) / 32767.0 * M_PI;
    */
    ai_cmd.local_target_speed[0] = two_to_float(&data_from_ether[0]) * 7.0;
    ai_cmd.local_target_speed[1] = two_to_float(&data_from_ether[2]) * 7.0;
    ai_cmd.global_vision_theta = two_to_float(&data_from_ether[4]) * M_PI;
    ai_cmd.target_theta = two_to_float(&data_from_ether[5]) * M_PI;

    if (data_from_ether[8] > 100) {
      ai_cmd.chip_en = 1;
      data_from_ether[8] = data_from_ether[8] - 100;
    } else {
      ai_cmd.chip_en = 0;
    }
    ai_cmd.kick_power = (float32_t)data_from_ether[8] / 20.0;
    ai_cmd.drible_power = (float32_t)data_from_ether[9] / 20.0;

    ai_cmd.keeper_en = data_from_ether[10];

    /* ai_cmd.global_ball_position[0] = ((int)(data_from_ether[11] << 8 | data_from_ether[12]) - 32767);
    ai_cmd.global_ball_position[1] = ((int)(data_from_ether[13] << 8 | data_from_ether[14]) - 32767);

    ai_cmd.global_robot_position[0] = ((int)(data_from_ether[15] << 8 | data_from_ether[16]) - 32767);
    ai_cmd.global_robot_position[1] = ((int)(data_from_ether[17] << 8 | data_from_ether[18]) - 32767);

    ai_cmd.global_global_target_position[0] = ((int)(data_from_ether[19] << 8 | data_from_ether[20]) - 32767);
    ai_cmd.global_global_target_position[1] = ((int)(data_from_ether[21] << 8 | data_from_ether[22]) - 32767);
    */

    ai_cmd.global_ball_position[0] = two_to_int(&data_from_ether[11]);
    ai_cmd.global_ball_position[1] = two_to_int(&data_from_ether[13]);
    ai_cmd.global_robot_position[0] = two_to_int(&data_from_ether[15]);
    ai_cmd.global_robot_position[1] = two_to_int(&data_from_ether[18]);
    ai_cmd.global_global_target_position[0] = two_to_int(&data_from_ether[19]);
    ai_cmd.global_global_target_position[1] = two_to_int(&data_from_ether[21]);

    ai_cmd.allow_local_feedback = data_from_ether[23];

    connection_check_cnt = 0;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart)
{
  if (huart->Instance == hlpuart1.Instance) {
    dma_printf_send_it(huart);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
    maintask_emargency();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
    actuator_buzzer(200, 200);
    // NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
