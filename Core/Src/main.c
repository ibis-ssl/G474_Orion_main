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
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>

#include "dma_printf.h"
#include "dma_scanf.h"
#include "management.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE * f)
#endif /* __GNUC__ */

void __io_putchar(uint8_t ch) { HAL_UART_Transmit(&hlpuart1, &ch, 1, 1); }

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

#define OMNI_OUTPUT_LIMIT (20)     //
#define OMNI_OUTPUT_GAIN_KP (150)  // ~ m/s / m : -250 -> 4cm : 1m/s
#define OMNI_OUTPUT_GAIN_KD (2)
#define OMNI_OUTPUT_GAIN_FF (1.0)

#define OMEGA_LIMIT (20.0)  // ~ rad/s
#define OMEGA_GAIN_KP (160.0)
#define OMEGA_GAIN_KD (4000.0)

#define ACCEL_LIMIT (6.0)       // m/ss, 4.5で滑る
#define ACCEL_LIMIT_BACK (3.0)  // m/ss
//const float OMNI_ROTATION_LENGTH = (0.07575);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs);
uint8_t getModeSwitch();
void maintask_run();
void sendRobotInfo();
void maintask_stop();
void motor_test();
void dribbler_test();
void kicker_test(bool manual_mode);
void send_accutuator_cmd_run();
void send_can_error();
void omniOdometory();
void mouseOdometory();
void yawFilter();
void resetLocalSpeedControl();
void resetAiCmdData();
uint32_t HAL_GetTick(void) { return uwTick; }
uint8_t decode_SW(uint16_t sw_raw_data);

// shared with other files
imu_t imu;
can_raw_t can_raw;
ai_cmd_t ai_cmd;
target_t target;
mouse_t mouse;
omni_t omni;
output_t output;
motor_t motor;
connection_t connection;
system_t sys;

UART_HandleTypeDef * huart_xprintf;

#define printf_BUF_SIZE 500
static char printf_buffer[printf_BUF_SIZE];

extern float32_t motor_voltage[4];  // for debug

struct
{
  volatile uint32_t main_loop_cnt;
  volatile float vel_radian;
  volatile bool print_flag;
} debug;

// communication with CM4
uint8_t data_from_ether[RX_BUF_SIZE_ETHER];
uint8_t tx_data_uart[TX_BUF_SIZE_ETHER];
uint8_t uart2_rx_it_buffer = 0;

#define AI_CMD_VEL_MAX_MPS (7.0)

#define FLAG_SSL_VISION_OK (0x01)
#define FLAG_ENABLE_LOCAL_VISION (0x08)
#define FLAG_ENABLE_KEEPER_MODE (0x02)

// main state
uint32_t adc_sw_data;
uint8_t send_no;

// kicker
volatile uint16_t kick_state;
volatile float diff_global[2], diff_local[2];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void p(const char * format, ...)
{
  va_list args;
  va_start(args, format);
  vsprintf(printf_buffer + strlen(printf_buffer), format, args);
  va_end(args);
}
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

  /* Configure the sys clock */
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
  send_no=0;
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

  for (int i = 0; i < 4; i++) {
    actuator_buzzer(20, 20);
  }

  morse_long();

  setbuf(stdin, NULL);
  setbuf(stdout, NULL);
  setbuf(stderr, NULL);
  dma_printf_init(&hlpuart1);
  dma_scanf_init(&hlpuart1);

  printf("start\r\n");
  HAL_UART_Init(&hlpuart1);
  HAL_UART_Init(&huart2);

  HAL_UART_Receive_IT(&huart2, &uart2_rx_it_buffer, 1);

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

  HAL_ADC_Start_DMA(&hadc5, &adc_sw_data, 1);

  actuator_power_ONOFF(0);
  HAL_Delay(20);

  actuator_motor1(0.0, 0.0);
  actuator_motor2(0.0, 0.0);
  actuator_motor3(0.0, 0.0);
  actuator_motor4(0.0, 0.0);
  actuator_motor5(0.0, 0.0);

  actuator_kicker(1, 1);
  actuator_kicker_voltage(150.0);
  actuator_power_param(1, 15.0);  // min voltage
  actuator_power_param(2, 35.0);  // max voltage
  actuator_power_param(3, 50.0);  // max can_raw.current
  actuator_power_param(4, 90.0);  // max temp(fet)
  actuator_power_param(5, 90.0);  // max temp(solenoid)

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
  ICM20602_init();
  ICM20602_IMU_calibration2();
  ICM20602_clearAngle();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
  actuator_power_ONOFF(1);

  for (int i = 0; i < 8; i++) {
    actuator_buzzer(20, 20);
  }
  sys.starting_status_flag = true;

  HAL_Delay(100);
  HAL_TIM_Base_Start_IT(&htim7);
  // TIM interrupt is TIM7 only.

  HAL_Delay(1000);
  sys.starting_status_flag = false;
  //target.velocity[1] = 1.0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 30:black 31:red 32:green 33:yellow 34:blue 35:magenta 36:cyan 37:white(default)
    debug.main_loop_cnt++;
    if (debug.print_flag) {
      debug.print_flag = false;

      // initialize
      printf_buffer[0] = 0;
      /*p("mode %2d ", sys.main_mode);
      if (can_raw.power_voltage[0] < 22) {
        p("\e[33mBatt=%3.1f\e[37m ", can_raw.power_voltage[0]);
      } else {

      }*/
      //p("theta %+4.0f ", ai_cmd.global_vision_theta * 180 / M_PI);
      p("yaw=%+6.1f ", imu.yaw_angle);
      p("Batt=%3.1f ", can_raw.power_voltage[0]);
      //p("tar=%+6.1f ", ai_cmd.target_theta * 180 / M_PI);
      //p("omega = %+5.1f ",omega);
      //p("vel rad %3.1f ", debug.vel_radian);
      //p(" motor0=%.3f motor1=%.3f motor2=%.3f motor3=%.3f ",can_raw.motor_feedback[0],can_raw.motor_feedback[1],can_raw.motor_feedback[2],can_raw.motor_feedback[3]);
      //p(" v0=%.3f v1=%.3f v2=%.3f v3=%.3f ",voltage[0],voltage[1],voltage[2],voltage[3]);
      //p(" i0=%+5.1f i1=%+5.1f i2=%+5.1f i3=%+5.1f ", can_raw.current[0], can_raw.current[1], can_raw.current[2], can_raw.current[3]);
      //p(" FET=%.3f C1=%.3f C2=%.3f ", can_raw.temperature[4], can_raw.temperature[5], can_raw.temperature[6]);
      //p(" Batt %3.1f Cap=%3.0f BattC %+6.1f ", can_raw.power_voltage[0], can_raw.power_voltage[6], can_raw.current[4]);
      //p(" omni.mouse:x=%+3d, y=%+3d ",omni.mouse[0],omni.mouse[1]);
      //p(" ENC %+4.1f %+4.1f %+4.1f %+4.1f ", motor.enc_angle[0], motor.enc_angle[1], motor.enc_angle[2], motor.enc_angle[3]);

      if (connection.connected_ai) {
        p("\e[32mcon %3d , %3.0f\e[37m ", connection.check_ver, connection.cmd_rx_frq);
      } else if (connection.connected_cm4) {
        p("\e[33mcon %3d , %3.0f\e[37m ", connection.check_ver, connection.cmd_rx_frq);
      } else {
        p("\e[31mcon %3d , %3.0f\e[37m ", connection.check_ver, connection.cmd_rx_frq);
      }
      p("vel X %+4.1f Y %+4.1f tharW %+6.1f ", ai_cmd.local_target_speed[0], ai_cmd.local_target_speed[1], ai_cmd.target_theta * 180 / M_PI);
      p("kick %3.2f chip %d dri %3.2f keeper %d local %d ", ai_cmd.kick_power, ai_cmd.chip_en, ai_cmd.drible_power, ai_cmd.keeper_mode_en_flag, ai_cmd.local_vision_en_flag);
      //p("grbl robot X %+5d Y %+5d W %+4.1f ", ai_cmd.global_robot_position[0], ai_cmd.global_robot_position[1], ai_cmd.global_vision_theta);
      //p("G-ball X %+5d Y %+5d ", ai_cmd.global_ball_position[0], ai_cmd.global_ball_position[1]);
      //p("G-tar X %+5d Y %+5d ", ai_cmd.global_target_position[0], ai_cmd.global_target_position[1]);

      //p("tarPos X %+8.3f Y %+8.3f ", target.position[0] * 1000, target.position[1] * 1000);

      //p("raw X %+8.3f Y %+8.3f  ", omni.odom_raw[0] * 1000, omni.odom_raw[1] * 1000);
      //p("PD %+5.2f  %+5.2f ", omni.robot_pos_diff[0], omni.robot_pos_diff[1]);
      //p("omni X %+8.3f Y %+8.3f  ", omni.odom[0] * 1000, omni.odom[1] * 1000);
      //p("M0 %+4.1f M1 %+4.1f M2 %+4.1f M3 %+4.1f ", motor_voltage[0] + 20, motor_voltage[1] + 20, motor_voltage[2] + 20, motor_voltage[3] + 20);
      //p("odom log X %+6.1f Y %+6.1f ", omni.odom_speed_log_total[0], omni.odom_speed_log_total[1]);
      //p("output x %+6.2f y %+6.2f ", output.velocity[0], output.velocity[1]);

      // omni odom
      //p("raw_odom X %+8.3f Y %+8.3f ", -mouse.raw_odom[0] * 1000, -mouse.raw_odom[1] * 1000);
      //p("mouse floor X %+8.3f Y %+8.3f ", -mouse.floor_odom[0] * 1000, -mouse.floor_odom[1] * 1000);
      //p("mouse X %+8.3f Y %+8.3f ", -mouse.odom[0] * 1000, -mouse.odom[1] * 1000);
      //p("Error X %+8.3f Y %+8.3f ", (omni.odom[0] + mouse.odom[0]) * 1000, (omni.odom[1] + mouse.odom[1]) * 1000);
      //p("diff X %+8.3f Y %+8.3f ", mouse.raw_diff[0] * 1000, mouse.raw_diff[1] * 1000);
      //p("mouseRaw X %+8.1f %+8.1f ", mouse.raw[0], mouse.raw[1]);
      //p("raw X %+4d %+4d %6d", mouse.raw[0], mouse.raw[1], mouse.quality);

      //p("Local %+8.1f %+8.1f ", (diff_local[0]) * 1000, (diff_local[1]) * 1000);
      //p("tarVel X %+8.1f Y %+8.1f ", target.velocity[0] * 1000, target.velocity[1] * 1000);
      //p("local X %+8.1f Y %+8.1f ", target.local_velocity[0] * 1000, target.local_velocity[1] * 1000);
      //p("speedX %+8.1f speedY %+8.1f ", omni.local_odom_speed[0] * 1000, omni.local_odom_speed[1] * 1000);
      //p("tar-c %+8.1f %+8.1f ", target.local_velocity_current[0] * 1000, target.local_velocity_current[1] * 1000);
      //p("limitX %+8.1f, limitY %+8.1f", output.accel_limit[0] * MAIN_LOOP_CYCLE * 50, output.accel_limit[1] * MAIN_LOOP_CYCLE * 50 + 10);
      //p("out local-c %+8.1f %+8.1f ", output.local_velocity_current[0] * 1000, output.local_velocity_current[1] * 1000);

      //p("ball_local x=%d y=%d radius=%d FPS=%d ", ball_local_x, ball_local_y, ball_local_radius, ball_local_FPS);
      //p("Raw %02x %02x %02x %02x ", data_from_ether[10], data_from_ether[11], data_from_ether[12], data_from_ether[13]);
      //p("%02x %02x %02x %02x ", data_from_ether[23], data_from_ether[24], data_from_ether[25], data_from_ether[26]);
      //p("txRaw %6.3f", imu.yaw_angle - ai_cmd.global_vision_theta);

      /*if (ai_cmd.vision_lost_flag) {
        p("\e[33mallow 0x%02x vision lost %d local en %d keeper en %d\e[37m ", ai_cmd.allow_local_flags, ai_cmd.vision_lost_flag, ai_cmd.local_vision_en_flag, ai_cmd.keeper_mode_en_flag);
      } else {
        p("\e[32mallow 0x%02x vision lost %d local en %d keeper en %d\e[31m ", ai_cmd.allow_local_flags, ai_cmd.vision_lost_flag, ai_cmd.local_vision_en_flag, ai_cmd.keeper_mode_en_flag);
      }
      if (sys.error_flag) {
        p("\e[31m error : 0x%02x 0x%02x 0x%02x 0x%02x\e[31m", can_raw.error_no[0], can_raw.error_no[1], can_raw.error_no[2], can_raw.error_no[3]);
      }*/

      p("\r\n");
      //p("loop %6d", debug.main_loop_cnt);
      HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)printf_buffer, strlen(printf_buffer));

      debug.main_loop_cnt = 0;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief sys Clock Configuration
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void resetLocalSpeedControl()
{
  for (int i = 0; i < 2; i++) {
    target.position[i] = omni.odom[i];
    ai_cmd.local_target_speed[i] = 0;
  }
  imu.yaw_angle = ai_cmd.target_theta;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  mouse.integral_loop_cnt++;
  // TIM interrupt is TIM7 only.
  static uint8_t pre_sw_mode, sw_mode;
  pre_sw_mode = sw_mode;
  sw_mode = getModeSwitch();

  if (sys.error_flag) {
    sys.main_mode = 9;
    resetLocalSpeedControl();

  } else if (sw_mode != pre_sw_mode || sys.starting_status_flag) {  // reset
    sys.main_mode = 7;
    resetLocalSpeedControl();

  } else {
    sys.main_mode = sw_mode;
  }

  yawFilter();  // mode == 2なら相補フィルタなし
  omniOdometory();

  switch (sys.main_mode) {
    case 0:  // with ai
    case 1:  //
      if (connection.connected_ai == false) {
        maintask_stop();
      } else {
        maintask_run();
      }
      break;
    case 2:  // local test mode
      maintask_run();
      break;

    case 3:  // motor test
      motor_test();
      break;

    case 4:  // drible test
      dribbler_test();
      break;

    case 5:  // kicker test (auto)
      kicker_test(false);
      break;

    case 6:  // kicker test (manual)
      kicker_test(true);
      break;

    case 9:  // error
      maintask_stop();
      send_can_error();
      break;

    default:
      maintask_stop();
      break;
  }
  //

  static bool buzzer_state = false;
  static uint32_t buzzer_cnt = 0;
  buzzer_cnt++;
  if (buzzer_cnt > 100) {
    buzzer_cnt = 0;

    if (can_raw.power_voltage[0] < 21.0 || sys.error_flag) {
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

  static uint32_t connection_check_cnt = 0;
  connection_check_cnt++;
  if (connection_check_cnt > MAIN_LOOP_CYCLE / 4) {  // 0.25s
    if (connection.check_ver != connection.check_pre) {
      connection.connected_ai = true;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    } else {
      connection.connected_ai = false;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
      resetAiCmdData();
    }
    connection_check_cnt = 0;
    connection.check_pre = connection.check_ver;
    connection.cmd_rx_frq = (float)connection.cmd_cnt * 4;  // 0.25s cycle -> x4
    connection.cmd_cnt = 0;

    if (connection.cmd_rx_frq > 0) {
      connection.connected_cm4 = true;
    } else {
      connection.connected_cm4 = false;
    }
  }

  // interrupt : 500Hz
  static uint16_t cnt_time_50Hz;
  cnt_time_50Hz++;
  if (cnt_time_50Hz > 10) {
    cnt_time_50Hz = 0;

    debug.print_flag = true;

    actuator_power_ONOFF(1);

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);

  }




}

uint8_t getModeSwitch()
{
  return 15 - (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) + (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) << 1) + (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) << 3) + (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) << 2));
}

void motor_test()
{
  if (decode_SW(adc_sw_data) & 0b00000001) {
    omni_move(2.0, 0.0, 0.0, 2.0);  // fwd
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (decode_SW(adc_sw_data) & 0b00000010) {
    omni_move(-2.0, 0.0, 0.0, 2.0);  // back
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (decode_SW(adc_sw_data) & 0b00000100) {
    omni_move(0.0, -2.0, 0.0, 2.0);  // left
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (decode_SW(adc_sw_data) & 0b00001000) {
    omni_move(0.0, 2.0, 0.0, 2.0);  // right
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else if (decode_SW(adc_sw_data) & 0b00010000) {
    omni_move(0.0, 0.0, 20.0, 2.0);  // spin
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else {
    omni_move(0.0, 0.0, 0.0, 0.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
  }
  actuator_motor5(0.0, 0.0);
}

void dribbler_test()
{
  if (decode_SW(adc_sw_data) & 0b00010000) {
    actuator_motor5(0.5, 1.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
  } else {
    actuator_motor5(0.0, 0.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
  }
  omni_move(0.0, 0.0, 0.0, 0.0);
}

void kicker_test(bool manual_mode)
{
  static bool dribbler_up = false;

  if (kick_state != 0) {
    if (kick_state > MAIN_LOOP_CYCLE / 2) {
      if (can_raw.ball_detection[0] == 0) {
        kick_state = 0;
      }
    } else {
      kick_state++;
    }
  }

  if (dribbler_up == false && decode_SW(adc_sw_data) & 0b00000100) {
    dribbler_up = true;
    actuator_dribbler_down();
  } else if (dribbler_up == true && decode_SW(adc_sw_data) & 0b00001000) {
    dribbler_up = false;
    actuator_dribbler_up();
  }

  if (decode_SW(adc_sw_data) & 0b00010000) {
    if (!manual_mode) {
      actuator_motor5(0.5, 1.0);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
    if (can_raw.ball_detection[0] == 1 || manual_mode) {
      if (kick_state == 0) {
        actuator_kicker(2, 0);  // straight
        actuator_kicker(3, 100);
        kick_state = 1;
      }
    }
  } else if (decode_SW(adc_sw_data) & 0b00000010) {
    if (!manual_mode) {
      actuator_motor5(0.5, 1.0);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
    if (can_raw.ball_detection[0] == 1 || manual_mode) {
      if (kick_state == 0) {
        actuator_kicker(2, 1);  // chip
        actuator_kicker(3, 255);
        kick_state = 1;
      }
    }
  } else {
    actuator_motor5(0.0, 0.0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
    actuator_kicker(1, 1);  // charge enable
    actuator_kicker_voltage(150.0);
  }
  omni_move(0.0, 0.0, 0.0, 0.0);
}

void yawFilter()
{
  imu.pre_yaw_angle_rad = imu.yaw_angle_rad;
  imu.pre_yaw_angle = imu.yaw_angle;

  ICM20602_read_IMU_data((float)1.0 / MAIN_LOOP_CYCLE, &(imu.yaw_angle));

  if (sys.main_mode < 2 && ai_cmd.vision_lost_flag == false) {
    // 相補フィルタ
    imu.yaw_angle = imu.yaw_angle - (getAngleDiff(imu.yaw_angle * PI / 180.0, ai_cmd.global_vision_theta) * 180.0 / PI) * 0.001;  // 0.001 : gain
  }
  imu.yaw_angle_rad = imu.yaw_angle * M_PI / 180;
}

void theta_control(/*float global_target_thera,float robot_gyro_theta*/)
{
  // PID
  output.omega = (getAngleDiff(ai_cmd.target_theta, imu.yaw_angle_rad) * OMEGA_GAIN_KP) - (getAngleDiff(imu.yaw_angle_rad, imu.pre_yaw_angle_rad) * OMEGA_GAIN_KD);

  if (output.omega > OMEGA_LIMIT) {
    output.omega = OMEGA_LIMIT;
  }
  if (output.omega < -OMEGA_LIMIT) {
    output.omega = -OMEGA_LIMIT;
  }
  // omega = 0;
  // out : omega
}

void speed_control()
{
  // グローバル→ローカル座標系

  target.local_velocity[0] = target.velocity[0] * cos(-imu.yaw_angle_rad) - target.velocity[1] * sin(-imu.yaw_angle_rad);
  target.local_velocity[1] = target.velocity[0] * sin(-imu.yaw_angle_rad) + target.velocity[1] * cos(-imu.yaw_angle_rad);

  diff_global[0] = omni.odom[0] - target.position[0];
  diff_global[1] = omni.odom[1] - target.position[1];

  diff_local[0] = diff_global[0] * cos(-imu.yaw_angle_rad) - diff_global[1] * sin(-imu.yaw_angle_rad);
  diff_local[1] = diff_global[0] * sin(-imu.yaw_angle_rad) + diff_global[1] * cos(-imu.yaw_angle_rad);

  // 500Hz, m/s -> m / cycle
  for (int i = 0; i < 2; i++) {
    // 加速度制限
    output.accel_limit[i] = ACCEL_LIMIT / MAIN_LOOP_CYCLE;
    if (target.local_velocity[i] < target.local_velocity_current[i] && i == 0) {  // バック時だけ加速度制限変更
      output.accel_limit[i] = ACCEL_LIMIT_BACK / MAIN_LOOP_CYCLE;
    }

    // 減速方向は摩擦を使えるので制動力上げる
    if (fabs(target.local_velocity[i]) < fabs(target.local_velocity_current[i])) {
      output.accel_limit[i] *= 3;
    }

    // 目標移動位置を追い越してしまっている場合。速度ではないのはノイズが多いから
    // ノイズ対策であまりodom情報でアップデートはできないが、最大加速度側を増やして追従する
    // local_velocityに対して追従するlocal_velocity_currentの追従を早める
    if (diff_local[i] > 0 && target.local_velocity[i] > 0) {
      output.accel_limit[i] *= 5;
    }
    if (diff_local[i] < 0 && target.local_velocity[i] < 0) {
      output.accel_limit[i] *= 5;
    }

    if (target.local_velocity[i] >= target.local_velocity_current[i]) {
      if (target.local_velocity_current[i] + output.accel_limit[i] > target.local_velocity[i]) {
        target.local_velocity_current[i] = target.local_velocity[i];
      } else {
        target.local_velocity_current[i] += output.accel_limit[i];
      }
    } else {
      if (target.local_velocity_current[i] - output.accel_limit[i] < target.local_velocity[i]) {
        target.local_velocity_current[i] = target.local_velocity[i];
      } else {
        target.local_velocity_current[i] -= output.accel_limit[i];
      }
    }
  }

  // ローカル→グローバル座標系
  output.local_velocity_current[0] = target.local_velocity_current[0] * cos(imu.yaw_angle_rad) - target.local_velocity_current[1] * sin(imu.yaw_angle_rad);
  output.local_velocity_current[1] = target.local_velocity_current[0] * sin(imu.yaw_angle_rad) + target.local_velocity_current[1] * cos(imu.yaw_angle_rad);
  target.position[0] += output.local_velocity_current[0] / MAIN_LOOP_CYCLE;  // speed to position
  target.position[1] += output.local_velocity_current[1] / MAIN_LOOP_CYCLE;  // speed to position

  // ここから位置制御
  for (int i = 0; i < 2; i++) {
    // targetとodomの差分に上限をつける(吹っ飛び対策)
    float odom_diff_max = (float)OMNI_OUTPUT_LIMIT / OMNI_OUTPUT_GAIN_KP;
    if (target.position[i] - omni.odom[i] > odom_diff_max) {
      target.position[i] = omni.odom[i] + odom_diff_max;
    } else if (target.position[i] - omni.odom[i] < -odom_diff_max) {
      target.position[i] = omni.odom[i] - odom_diff_max;
    }

    // odom基準の絶対座標系
    omni.odom_floor_diff[i] = omni.odom[i] - target.position[i];
  }

  // グローバル→ローカル座標系
  omni.robot_pos_diff[0] = omni.odom_floor_diff[0] * cos(-imu.yaw_angle_rad) - omni.odom_floor_diff[1] * sin(-imu.yaw_angle_rad);
  omni.robot_pos_diff[1] = omni.odom_floor_diff[0] * sin(-imu.yaw_angle_rad) + omni.odom_floor_diff[1] * cos(-imu.yaw_angle_rad);

  output.velocity[0] = -omni.robot_pos_diff[0] * OMNI_OUTPUT_GAIN_KP - omni.local_odom_speed[0] * OMNI_OUTPUT_GAIN_KD + target.local_velocity[0] * OMNI_OUTPUT_GAIN_FF;
  output.velocity[1] = -omni.robot_pos_diff[1] * OMNI_OUTPUT_GAIN_KP - omni.local_odom_speed[1] * OMNI_OUTPUT_GAIN_KD + target.local_velocity[1] * OMNI_OUTPUT_GAIN_FF;
}

void output_limit()
{
  float limit_gain = 0;
  if (output.velocity[0] > OMNI_OUTPUT_LIMIT) {
    limit_gain = output.velocity[0] / OMNI_OUTPUT_LIMIT;
    output.velocity[0] = OMNI_OUTPUT_LIMIT;
    output.velocity[1] /= limit_gain;
  } else if (output.velocity[0] < -OMNI_OUTPUT_LIMIT) {
    limit_gain = -output.velocity[0] / OMNI_OUTPUT_LIMIT;
    output.velocity[0] = -OMNI_OUTPUT_LIMIT;
    output.velocity[1] /= limit_gain;
  }

  if (output.velocity[1] > OMNI_OUTPUT_LIMIT) {
    limit_gain = output.velocity[1] / OMNI_OUTPUT_LIMIT;
    output.velocity[1] = OMNI_OUTPUT_LIMIT;
    output.velocity[0] /= limit_gain;
  } else if (output.velocity[1] < -OMNI_OUTPUT_LIMIT) {
    limit_gain = -output.velocity[1] / OMNI_OUTPUT_LIMIT;
    output.velocity[1] = -OMNI_OUTPUT_LIMIT;
    output.velocity[0] /= limit_gain;
  }
}

void maintask_run()
{
  if (ai_cmd.local_vision_en_flag == false) {
    target.velocity[0] = ai_cmd.local_target_speed[0];
    target.velocity[1] = ai_cmd.local_target_speed[1];
  } else {
    //
    target.velocity[0] = 0;
    target.velocity[1] = 0;
  }

  // デバッグ用に勝手に動くやつ
  /*if (sys.main_mode == 2) {
    debug.vel_radian += (float)5 / 10 * 2 * M_PI / MAIN_LOOP_CYCLE;

    if (debug.vel_radian > M_PI * 2) {
      debug.vel_radian = 0;
    }
    if (sin(debug.vel_radian) > sin(M_PI / 4)) {
      target.velocity[0] = +0.5;
    } else if (sin(debug.vel_radian) < -sin(M_PI / 4)) {
      target.velocity[0] = -0.5;
    } else {
      target.velocity[0] = 0;
    }
    if (cos(debug.vel_radian) > sin(M_PI / 4)) {
      target.velocity[1] = +0.3;
    } else if (cos(debug.vel_radian) < -sin(M_PI / 4)) {
      target.velocity[1] = -0.3;
    } else {
      target.velocity[1] = 0;
    }

    ai_cmd.target_theta = sin(debug.vel_radian);
  }  //*/

  speed_control();
  output_limit();
  theta_control();

  omni_move(output.velocity[0], output.velocity[1], output.omega, OMNI_OUTPUT_LIMIT);
  send_accutuator_cmd_run();
}

void send_accutuator_cmd_run()
{
  if (ai_cmd.kick_power > 0) {
    if (kick_state == 0) {
      if (can_raw.ball_detection[0] == 1) {
        uint8_t kick_power_param = (float)ai_cmd.kick_power * 255.0;
        printf(" kick=%d\r\n", kick_power_param);
        actuator_kicker(3, (uint8_t)kick_power_param);
        kick_state = 1;
      }
    } else {
      if (kick_state > MAIN_LOOP_CYCLE / 2) {
        if (can_raw.ball_detection[0] == 0) {
          kick_state = 0;
        }
      } else {
        kick_state++;
      }
    }
  }

  static uint8_t can_sending_index = 0;

  can_sending_index++;
  switch (can_sending_index) {
    case 1:
      if (ai_cmd.chip_en == true) {
        actuator_kicker(2, 1);
      } else {
        actuator_kicker(2, 0);
      }
      break;

    case 2:
      if (ai_cmd.chip_en == true) {
        actuator_dribbler_up();
      } else {
        actuator_dribbler_down();
      }
      break;

    case 3:
      actuator_kicker(1, 1);
      break;

    case 4:
      actuator_kicker_voltage(150.0);
      break;

    case 5:
      actuator_motor5(ai_cmd.drible_power, 1.0);
      break;

    default:
      can_sending_index = 0;
      break;
  }
}

void sendRobotInfo()
{
	  tx_msg_t msg;
	  static uint8_t ring_counter = 0;
	  ring_counter++;
	  if (ring_counter > 200) {
	    ring_counter = 0;
	  }
	  char* temp;

	  uint8_t senddata[16];

	  switch (send_no) {
		case 0:
		  senddata[0]=0xFA;
		  senddata[1]=0xFB;
		  senddata[2]=send_no+10;
		  senddata[3]=ring_counter;
		  	  temp = (char*)&imu.yaw_angle;
		  senddata[4]=temp[0];
		  senddata[5]=temp[1];
		  senddata[6]=temp[2];
		  senddata[7]=temp[3];
		  	  msg.data.diff_angle = imu.yaw_angle - ai_cmd.global_vision_theta;
		  	  temp = (char*)&msg.data.diff_angle;
		  senddata[8]=temp[0];
		  senddata[9]=temp[1];
		  senddata[10]=temp[2];
		  senddata[11]=temp[3];
		  senddata[12]=can_raw.ball_detection[0];
		  senddata[13]=can_raw.ball_detection[1];
		  senddata[14]=can_raw.ball_detection[2];
		  senddata[15]=can_raw.ball_detection[3];
		break;
		case 1:
		  senddata[0]=0xFA;
		  senddata[1]=0xFB;
		  senddata[2]=send_no+10;
		  senddata[3]=ring_counter;
		  senddata[4]=can_raw.error_no[0];
		  senddata[5]=can_raw.error_no[1];
		  senddata[6]=can_raw.error_no[2];
		  senddata[7]=can_raw.error_no[3];
		  senddata[8]=can_raw.error_no[4];
		  senddata[9]=can_raw.error_no[5];
		  senddata[10]=can_raw.error_no[6];
		  senddata[11]=can_raw.error_no[7];
		  senddata[12]=(uint8_t)can_raw.current[0];
		  senddata[13]=(uint8_t)can_raw.current[1];
		  senddata[14]=(uint8_t)can_raw.current[2];
		  senddata[15]=(uint8_t)can_raw.current[3];
		break;
		case 2:
		  senddata[0]=0xFA;
		  senddata[1]=0xFB;
		  senddata[2]=send_no+10;
		  senddata[3]=ring_counter;
		  senddata[4]=kick_state / 10;
		  senddata[5]=(uint8_t)can_raw.temperature[0];
		  senddata[6]=(uint8_t)can_raw.temperature[1];
		  senddata[7]=(uint8_t)can_raw.temperature[2];
		  senddata[8]=(uint8_t)can_raw.temperature[3];
		  senddata[9]=(uint8_t)can_raw.temperature[4];
		  senddata[10]=(uint8_t)can_raw.temperature[5];
		  senddata[11]=(uint8_t)can_raw.temperature[6];
		  	  temp = (char*)&(can_raw.power_voltage[0]);
		  senddata[12]=temp[0];
		  senddata[13]=temp[1];
		  senddata[14]=temp[2];
		  senddata[15]=temp[3];
		break;
		case 3:
		  senddata[0]=0xFA;
		  senddata[1]=0xFB;
		  senddata[2]=send_no+10;
	      senddata[3]=ring_counter;
	  	  	  temp = (char*)&(can_raw.power_voltage[6]);
		  senddata[4]=temp[0];
		  senddata[5]=temp[1];
		  senddata[6]=temp[2];
		  senddata[7]=temp[3];
	  	  	  temp = (char*)&omni.odom[0];
		  senddata[8]=temp[0];
		  senddata[9]=temp[1];
		  senddata[10]=temp[2];
		  senddata[11]=temp[3];
	  	  	  temp = (char*)&omni.odom[1];
	      senddata[11]=temp[0];
	      senddata[12]=temp[1];
	      senddata[13]=temp[2];
	      senddata[14]=temp[3];
		break;
		case 4:
		  senddata[0]=0xFA;
		  senddata[1]=0xFB;
		  senddata[2]=send_no+10;
	      senddata[3]=ring_counter;
	  	  	  temp = (char*)&omni.odom_speed[0];
		  senddata[4]=temp[0];
		  senddata[5]=temp[1];
		  senddata[6]=temp[2];
		  senddata[7]=temp[3];
	  	  	  temp = (char*)&omni.odom_speed[1];
		  senddata[8]=temp[0];
		  senddata[9]=temp[1];
		  senddata[10]=temp[2];
		  senddata[11]=temp[3];
	      senddata[11]=connection.check_ver;
	      senddata[12]=0;
	      senddata[13]=0;
	      senddata[14]=0;
		break;
		default:
		  senddata[0]=0xFA;
		  senddata[1]=0xFB;
		  senddata[2]=send_no+100;
		  senddata[3]=ring_counter;
		  senddata[4]=connection.check_ver;
		  senddata[5]=0;
		  senddata[6]=0;
		  senddata[7]=0;
		  senddata[8]=0;
		  senddata[9]=0;
		  senddata[10]=0;
		  senddata[11]=0;
		  senddata[12]=0;
		  senddata[13]=0;
		  senddata[14]=0;
		  senddata[15]=0;
			break;
	}
	  send_no++;
	  if(send_no>4){send_no=0;}

	  HAL_UART_Transmit(&huart2, senddata, sizeof(senddata),0xff);

}

void maintask_stop()
{
  omni_move(0.0, 0.0, 0.0, 0.0);
  actuator_motor5(0.0, 0.0);
  actuator_kicker(1, 0);
  actuator_kicker_voltage(0.0);
  actuator_dribbler_down();
}

void send_can_error()
{
  uint8_t senddata_error[8];
  can1_send(0x000, senddata_error);
  can2_send(0x000, senddata_error);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
}
void resetAiCmdData()
{
  ai_cmd.local_target_speed[0] = 0;
  ai_cmd.local_target_speed[1] = 0;
  ai_cmd.global_vision_theta = 0;
  ai_cmd.target_theta = 0;
  ai_cmd.chip_en = false;
  ai_cmd.kick_power = 0;
  ai_cmd.drible_power = 0;
  ai_cmd.allow_local_flags = 0;

  ai_cmd.global_ball_position[0] = 0;
  ai_cmd.global_ball_position[1] = 0;
  ai_cmd.global_robot_position[0] = 0;
  ai_cmd.global_robot_position[1] = 0;
  ai_cmd.global_target_position[0] = 0;
  ai_cmd.global_target_position[1] = 0;

  ai_cmd.ball_local_x = 0;
  ai_cmd.ball_local_y = 0;
  ai_cmd.ball_local_radius = 0;
  ai_cmd.ball_local_FPS = 0;

  ai_cmd.vision_lost_flag = true;
  ai_cmd.local_vision_en_flag = false;
  ai_cmd.keeper_mode_en_flag = false;
}

void parseRxCmd()
{
  connection.cmd_cnt++;
  connection.check_ver = data_from_ether[1];

  // time out
  if (connection.connected_ai == 0) {
    resetAiCmdData();
    return;
  }

  ai_cmd.local_target_speed[0] = two_to_float(&data_from_ether[2]) * AI_CMD_VEL_MAX_MPS;
  ai_cmd.local_target_speed[1] = two_to_float(&data_from_ether[4]) * AI_CMD_VEL_MAX_MPS;
  ai_cmd.global_vision_theta = two_to_float(&data_from_ether[6]) * M_PI;
  ai_cmd.target_theta = two_to_float(&data_from_ether[8]) * M_PI;
  if (data_from_ether[10] > 100) {
    ai_cmd.chip_en = true;
    ai_cmd.kick_power = (float)(data_from_ether[10] - 100) / 20;
  } else {
    ai_cmd.kick_power = (float)data_from_ether[10] / 20;
    ai_cmd.chip_en = false;
  }
  ai_cmd.drible_power = (float)data_from_ether[11] / 20;

  ai_cmd.allow_local_flags = data_from_ether[12];

  ai_cmd.global_ball_position[0] = two_to_int(&data_from_ether[13]);
  ai_cmd.global_ball_position[1] = two_to_int(&data_from_ether[15]);
  ai_cmd.global_robot_position[0] = two_to_int(&data_from_ether[17]);
  ai_cmd.global_robot_position[1] = two_to_int(&data_from_ether[19]);
  ai_cmd.global_target_position[0] = two_to_int(&data_from_ether[21]);
  ai_cmd.global_target_position[1] = two_to_int(&data_from_ether[23]);

  ai_cmd.ball_local_x = data_from_ether[RX_BUF_SIZE_ETHER - 8] << 8 | data_from_ether[RX_BUF_SIZE_ETHER - 7];
  ai_cmd.ball_local_y = data_from_ether[RX_BUF_SIZE_ETHER - 6] << 8 | data_from_ether[RX_BUF_SIZE_ETHER - 5];
  ai_cmd.ball_local_radius = data_from_ether[RX_BUF_SIZE_ETHER - 4] << 8 | data_from_ether[RX_BUF_SIZE_ETHER - 3];
  ai_cmd.ball_local_FPS = data_from_ether[RX_BUF_SIZE_ETHER - 2];

  if ((ai_cmd.allow_local_flags & FLAG_SSL_VISION_OK) != 0) {
    ai_cmd.vision_lost_flag = false;
  } else {
    ai_cmd.vision_lost_flag = true;
  }

  if ((ai_cmd.allow_local_flags & FLAG_ENABLE_LOCAL_VISION) != 0) {
    ai_cmd.local_vision_en_flag = true;
  } else {
    ai_cmd.local_vision_en_flag = false;
  }

  if ((ai_cmd.allow_local_flags & FLAG_ENABLE_KEEPER_MODE) != 0) {
    ai_cmd.keeper_mode_en_flag = true;
  } else {
    ai_cmd.keeper_mode_en_flag = false;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  static int32_t uart_rx_cmd_idx = 0;
  uint8_t rx_data_tmp;

  if (huart->Instance == USART2) {
    rx_data_tmp = uart2_rx_it_buffer;
    HAL_UART_Receive_IT(&huart2, &uart2_rx_it_buffer, 1);

    if (uart_rx_cmd_idx >= 0 && uart_rx_cmd_idx < RX_BUF_SIZE_ETHER) {
      data_from_ether[uart_rx_cmd_idx] = rx_data_tmp;
    }

    // look up header byte
    if (uart_rx_cmd_idx == -1 && rx_data_tmp == 254) {
      uart_rx_cmd_idx++;
    }

    // data byte
    if (uart_rx_cmd_idx != -1 && uart_rx_cmd_idx < RX_BUF_SIZE_ETHER) {
      uart_rx_cmd_idx++;
    }

    // end
    if (uart_rx_cmd_idx == RX_BUF_SIZE_ETHER) {
      uart_rx_cmd_idx = -1;
      parseRxCmd();
      sendRobotInfo();
    }
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
    maintask_stop(255, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
    actuator_buzzer(200, 200);
    // NVIC_sysReset();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t * file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
