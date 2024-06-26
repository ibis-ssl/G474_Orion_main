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
#include <math.h>
#include <stdarg.h>

#include "actuator.h"
#include "ai_comm.h"
#include "buzzer_control.h"
#include "can_ibis.h"
#include "icm20602_spi.h"
#include "management.h"
#include "odom.h"
#include "omni_wheel.h"
#include "ring_buffer.h"
#include "robot_control.h"
#include "test_func.h"
#include "util.h"

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

#define OMNI_OUTPUT_LIMIT (40)  //上げると過電流エラーになりがち

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef * hfdcan);
uint8_t getModeSwitch();
void maintask_run();
void yawFilter();
bool allEncInitialized();
uint32_t HAL_GetTick(void) { return uwTick; }

// shared with other files
imu_t imu;
can_raw_t can_raw;
ai_cmd_t ai_cmd, ai_cmd_buf;
target_t target;
mouse_t mouse;
omni_t omni;
output_t output;
motor_t motor;
connection_t connection;
system_t sys;
integration_control_t integ;
accel_vector_t acc_vel;
debug_t debug;

UART_HandleTypeDef * huart_xprintf;

#define printf_BUF_SIZE 500
static char printf_buffer[printf_BUF_SIZE];

struct
{
  float spin_total[4];
  float diff[4];
} slip_detect;

struct
{
  bool enabled_flag;
} latency_test;

// communication with CM4
uint8_t data_from_cm4[RX_BUF_SIZE_ETHER];
uint8_t tx_data_uart[TX_BUF_SIZE_ETHER];
uint8_t uart2_rx_it_buffer = 0, lpuart1_rx_it_buffer = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void __io_putchar(uint8_t ch) { HAL_UART_Transmit(&hlpuart1, &ch, 1, 1); }

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
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_ADC5_Init();
  MX_ADC3_Init();
  MX_FDCAN2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  integ.odom_log[0] = initRingBuffer(SPEED_LOG_BUF_SIZE);
  integ.odom_log[1] = initRingBuffer(SPEED_LOG_BUF_SIZE);
  omni.local_speed_log[0] = initRingBuffer(SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE);
  omni.local_speed_log[1] = initRingBuffer(SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE);

  // 本来はリアルタイムに更新できた方が良いが、まだそのシステムがないので固定値
  ai_cmd.latency_time_ms = 30;
  //

  sys.kick_state = 0;
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

  actuator_buzzer_frq(1046, 50);  //C5
  actuator_buzzer_frq(1174, 50);  //D5
  actuator_buzzer_frq(1318, 50);  //E5
  actuator_buzzer_frq(1396, 50);  //F5

  setbuf(stdin, NULL);
  setbuf(stdout, NULL);
  setbuf(stderr, NULL);

  HAL_UART_Init(&huart2);
  HAL_UART_Receive_IT(&huart2, &uart2_rx_it_buffer, 1);

  HAL_UART_Init(&hlpuart1);
  HAL_UART_Receive_IT(&hlpuart1, &lpuart1_rx_it_buffer, 1);

  printf("orion main start %s %s\r\n", __DATE__, __TIME__);

  HAL_ADC_Start_DMA(&hadc5, &sys.sw_data, 1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
  ICM20602_init();
  ICM20602_init();
  ICM20602_IMU_calibration2();
  ICM20602_clearAngle();

  printf("\n\rcomplete imu init\r\n");
  // CANより先にIMUのキャリブレーションする

  printf("\n\rstart can1\r\n");
  can1_init_ibis(&hfdcan1);

  printf("\n\rstart can2\r\n");
  can2_init_ibis(&hfdcan2);

  actuator_power_ONOFF(0);
  HAL_Delay(20);

  actuator_motor1(0.0, 0.0);
  actuator_motor2(0.0, 0.0);
  actuator_motor3(0.0, 0.0);
  actuator_motor4(0.0, 0.0);
  actuator_motor5(0.0, 0.0);

  actuator_kicker(1, 1);
  actuator_kicker_voltage(0.0);
  actuator_power_param(1, 15.0);  // min voltage
  actuator_power_param(2, 35.0);  // max voltage
  actuator_power_param(3, 50.0);  // max can_raw.current
  actuator_power_param(4, 90.0);  // max temp(fet)
  actuator_power_param(5, 90.0);  // max temp(solenoid)

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

  actuator_power_ONOFF(1);

  actuator_buzzer_frq(1046, 50);  //C5
  actuator_buzzer_frq(1174, 50);  //D5
  actuator_buzzer_frq(1046, 50);  //C5
  actuator_buzzer_frq(1174, 50);  //D5
  actuator_buzzer_frq(1318, 50);  //E5
  actuator_buzzer_frq(1396, 50);  //F5

  actuator_power_ONOFF(1);

  sys.system_time_ms = 1000;                               //
  sys.stop_flag_request_time = sys.system_time_ms + 1000;  // !!注意!! TIM7の割り込みがはじまってから1000ms間停止
  connection.already_connected_ai = false;
  HAL_Delay(100);
  HAL_TIM_Base_Start_IT(&htim7);
  // TIM interrupt is TIM7 only.

  HAL_Delay(500);
  debug.print_idx = 4;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    debug.main_loop_cnt++;
    if (debug.print_flag /* && fabs(omni.local_odom_speed_mvf[0]) > 0.01*/) {
      debug.print_flag = false;

      // 文字列初期化
      printf_buffer[0] = 0;

      // 文字色メモ
      // 30:black 31:red 32:green 33:yellow 34:blue 35:magenta 36:cyan 37:white(default)
      p("\e[0m");
      //p("yaw=%+6.1f ", imu.yaw_angle);
      if (can_raw.power_voltage[5] < LOW_VOLTAGE_LIMIT) {
        p("\e[33mBatt=%3.1f\e[37m ", can_raw.power_voltage[5]);
      } else {
        p("Batt=%3.1f ", can_raw.power_voltage[5]);
      }
      debug.out_total_spin = output.motor_voltage[0] + output.motor_voltage[1] + output.motor_voltage[2] + output.motor_voltage[3];
      debug.fb_total_spin = (can_raw.motor_feedback[0] + can_raw.motor_feedback[1] + can_raw.motor_feedback[2] + can_raw.motor_feedback[3]) / 1.5;
      debug.true_yaw_speed = imu.yaw_angle - debug.pre_yaw_angle;

      if (fabs(debug.true_yaw_speed - debug.true_fb_total_spin) > 100 && fabs(ai_cmd.target_theta - imu.yaw_angle) > 5) {
        //actuator_buzzer_frq_on(1046);
        //debug.acc_step_down_flag = true;

      } else {
        //actuator_buzzer_off();
        debug.acc_step_down_flag = false;
      }

      if (sys.main_mode == MAIN_MODE_ERROR) {
        p("\e[31m error : ID %5d / Info %5d / Value %+8.3f \e[31m", sys.error_id, sys.error_info, sys.error_value);
      }
      if (sys.system_time_ms < sys.stop_flag_request_time) {
        p("\e[33m");
      }

      switch (debug.print_idx) {
        case 0:
          p("CMD ");
          // 通信接続状態表示
          if (connection.connected_ai) {
            p("\e[32m%3d,%3.0f\e[37m ", connection.check_ver, connection.cmd_rx_frq);
          } else if (connection.connected_cm4) {
            p("\e[33m%3d,%3.0f\e[37m ", connection.check_ver, connection.cmd_rx_frq);
          } else {
            p("\e[31m%3d,%3.0f\e[37m ", connection.check_ver, connection.cmd_rx_frq);
          }

          p("AIcmd Vx %+4.1f Vy %+4.1f Tw %+6.1f ", ai_cmd.local_target_speed[0], ai_cmd.local_target_speed[1], ai_cmd.target_theta * 180 / M_PI);
          if (sys.main_mode != MAIN_MODE_CMD_DEBUG_MODE) {
            if (ai_cmd.vision_lost_flag) {  // SSL-Vision (Robot)
              p("\e[33m");
            }
            p("Vision X %+6.1f Y %+6.1f W %+4.1f ", ai_cmd.global_robot_position[0] * 1000, ai_cmd.global_robot_position[1] * 1000, ai_cmd.global_vision_theta);
            p("AIcmd X %+6.2f Y %+6.2f ", ai_cmd.global_target_position[0], ai_cmd.global_target_position[1]);
            p("Wdidd %+5.1f", (getAngleDiff(imu.yaw_angle * M_PI / 180.0, ai_cmd.global_vision_theta) * 180 / M_PI));
            p("\e[37m ");  // end color
            p("update %d ", debug.theta_override_flag);
          } else {
            p("omni X %+8.3f Y %+8.3f ", omni.odom[0] * 1000, omni.odom[1] * 1000);
            p("Wdidd %+5.1f", (getAngleDiff(imu.yaw_angle * M_PI / 180.0, ai_cmd.global_vision_theta) * 180 / M_PI));
            p("Temp %3.0f %3.0f %3.0f %3.0f", can_raw.temperature[0], can_raw.temperature[1], can_raw.temperature[2], can_raw.temperature[3]);
          }

          break;
        case 1:  //Motor
          p("MOTOR ");
          p("Spd M0=%+6.1f M1=%+6.1f M2=%+6.1f M3=%+6.1f / ", can_raw.motor_feedback[0], can_raw.motor_feedback[1], can_raw.motor_feedback[2], can_raw.motor_feedback[3]);
          p("Pw v0=%5.1f v1=%5.1f v2=%5.1f v3=%5.1f / ", can_raw.power_voltage[0], can_raw.power_voltage[1], can_raw.power_voltage[2], can_raw.power_voltage[3]);
          p("Im i0=%+5.1f i1=%+5.1f i2=%+5.1f i3=%+5.1f / ", can_raw.current[0], can_raw.current[1], can_raw.current[2], can_raw.current[3]);
          p("Temp m0=%3.0f m1=%3.0f m2=%3.0f m3=%3.0f ", can_raw.temperature[0], can_raw.temperature[1], can_raw.temperature[2], can_raw.temperature[3]);

          break;
        case 2:  // Dribblerテスト
          p("DRIBBLER ");
          p("Batt(Sub) %3.1f / ", can_raw.power_voltage[4]);
          p("ball_sensor %d %d / ESC Spd %+5.0f / ", can_raw.ball_detection[0], can_raw.ball_detection[1], can_raw.motor_feedback_velocity[4]);
          p("local_vision x=%3d y=%3d radius=%3d FPS=%3d ", ai_cmd.ball_local_x, ai_cmd.ball_local_y, ai_cmd.ball_local_radius, ai_cmd.ball_local_FPS);

          break;
        case 3:  // Kicker Test
          p("KICKER ");
          p("Batt(Pw) %3.1f Cap=%3.0f BattC %+6.1f Batt(Sub) %3.1f / ", can_raw.power_voltage[5], can_raw.power_voltage[6], can_raw.current[4], can_raw.power_voltage[4]);
          p("FET=%5.1f L1=%5.1f L2=%5.1f / ", can_raw.temperature[4], can_raw.temperature[5], can_raw.temperature[6]);
          p("ball_sensor %d %d / ESC Spd %+5.0f ", can_raw.ball_detection[0], can_raw.ball_detection[1], can_raw.motor_feedback_velocity[4]);

          break;
        case 4:  // Mouse odom
          p("MOUSE ");
          p("%d / %d %d %d %d", allEncInitialized(), can_raw.enc_rx_flag[0], can_raw.enc_rx_flag[1], can_raw.enc_rx_flag[2], can_raw.enc_rx_flag[3]);
          //p("raw_odom X %+8.3f Y %+8.3f ", mouse.raw_odom[0] * 1000, mouse.raw_odom[1] * 1000);
          //p("mouse floor X %+8.3f Y %+8.3f ", mouse.floor_odom[0] * 1000, mouse.floor_odom[1] * 1000);
          p("omni-odom X %+8.1f ,Y %+8.1f. ", omni.odom[0] * 1000, omni.odom[1] * 1000);
          p("mouse X %+8.2f Y %+8.2f ", mouse.odom[0] * 1000, mouse.odom[1] * 1000);
          p("Error X %+8.2f Y %+8.2f ", (mouse.odom[0] - omni.odom[0]) * 1000, (mouse.odom[1] - omni.odom[1]) * 1000);
          p("diff X %+8.2f Y %+8.2f ", mouse.raw_diff[0] * 1000, mouse.raw_diff[1] * 1000);
          //p("mouseRaw X %+8.1f Y %+8.1f ", mouse.raw[0], mouse.raw[1]);
          p("raw X %+4d Y %+4d Q %6d", mouse.raw[0], mouse.raw[1], mouse.quality);

          break;
        case 5:
          p("ODOM ");
          p("omg %+3.0f ", output.omega);
          //p("ENC angle %+6.3f %+6.3f %+6.3f %+6.3f ", motor.enc_angle[0], motor.enc_angle[1], motor.enc_angle[2], motor.enc_angle[3]);
          //p("omni-odom X %+8.1f ,Y %+8.1f. ", omni.odom[0] * 1000, omni.odom[1] * 1000);
          //p("cnt %4d ", connection.vision_update_cycle_cnt);
          //p("integ %+5.2f %+5.2f ", integ.vision_based_position[0], integ.vision_based_position[1]);
          //p("integ-diffG %+5.2f %+5.2f ", integ.position_diff[0], integ.position_diff[1]);
          p("AI X %+4.1f Y %+4.1f ", ai_cmd.local_target_speed[0], ai_cmd.local_target_speed[1]);
          p("integ-diff %+6.3f %+6.3f ", integ.local_target_diff[0], integ.local_target_diff[1]);
          //p("tar-pos X %+8.1f, Y %+8.1f ", target.global_vel_now[0], target.global_vel_now[1]);
          p("cmd-vel %+5.2f, %+5.2f, ", target.velocity[0], target.velocity[1]);
          p("vel-now %+6.3f, %+6.3f, ", target.local_vel_now[0], target.local_vel_now[1]);
          p("acc X %+8.2f, Y %+8.2f, ", output.accel[0], output.accel[1]);
          //p("vel-diff X %+8.2f, Y %+8.2f, ", acc_vel.vel_error_xy[0] * 1000, acc_vel.vel_error_xy[1] * 1000);
          //p("rad %+8.2f, scalar %+8.2f, ", acc_vel.vel_error_rad * 180 / M_PI, acc_vel.vel_error_scalar * 1000);
          p("vcp-d X %+5.3f, Y %+5.3f, ", omni.robot_pos_diff[0], omni.robot_pos_diff[1]);  // x150は出力ゲイン
          //p("FF-N %+5.1f FF-T %+5.1f ", target.local_vel_ff_factor[0], target.local_vel_ff_factor[1]);
          p("out-vel %+5.1f, %+5.1f, ", output.velocity[0], output.velocity[1]);
          //p("acc sca %7.4f rad %5.2f ", acc_vel.vel_error_scalar, acc_vel.vel_error_rad);
          p("real-vel X %+8.3f, Y %+8.3f, ", omni.local_odom_speed_mvf[0], omni.local_odom_speed_mvf[1]);
          //p("odom X %+8.3f, Y %+8.3f, ", omni.odom[0], omni.odom[1]);

          //p("vel-diff %+8.3f, %+8.3f, ", target.local_vel_now[0] - omni.local_odom_speed_mvf[0], target.local_vel_now[1] - omni.local_odom_speed_mvf[1]);

          //p("M0 %+5.2f M1 %+5.2f M2 %+5.2f M3 %+5.2f ", output.motor_voltage[0], output.motor_voltage[1], output.motor_voltage[2], output.motor_voltage[3]);

          break;
        case 6:
          p("CMD ");
          p("ck%3d cnt %3d main %6d CR 0x%4x", connection.check_ver, debug.uart_rx_itr_cnt, debug.main_loop_cnt / 10, huart2.Instance->CR1);
          p("AI X %+4.1f Y %+4.1f ", ai_cmd.local_target_speed[0], ai_cmd.local_target_speed[1]);
          p("scl %4.1f ", ai_cmd.local_target_speed_scalar);
          p("TPx %+4.1f TPy %+4.1f TW %+6.1f ", ai_cmd.global_target_position[0], ai_cmd.global_target_position[1], ai_cmd.target_theta * 180 / M_PI);
          p("Vis Gbrl-rb X %+6.2f Y %+6.2f Theta %+6.1f ", ai_cmd.global_robot_position[0], ai_cmd.global_robot_position[1], ai_cmd.global_vision_theta);
          //p("Gbrl-ball X %+6.2f Y %+6.2f ", ai_cmd.global_ball_position[0], ai_cmd.global_ball_position[1]);
          p("lst %d stp %d kic %3.2f chp %d dri %3.2f kpr %d lcl %d ", ai_cmd.vision_lost_flag, ai_cmd.stop_request_flag, ai_cmd.kick_power, ai_cmd.chip_en, ai_cmd.dribble_power,
            ai_cmd.keeper_mode_en_flag, ai_cmd.local_vision_en_flag);
          break;
        case 7:
          p("LATENCY ");
          p("EN%d cnt %4d target %+5.2f diff %+5.2f", debug.latency_check_enabled, debug.latency_check_seq_cnt, debug.rotation_target_theta,
            getAngleDiff(debug.rotation_target_theta, imu.yaw_angle_rad));
          break;
        case 8:
          p("SYSTEM TIME ");
          for (int i = 0; i < 7; i++) {
            p("%4d ", debug.start_time[i]);
          }
          p(" can enc rx : ");
          for (int i = 0; i < BOARD_ID_MAX; i++) {
            p("%6d ", can_raw.board_rx_timeout_cnt[i]);
          }
          break;
        default:
          debug.print_idx = 0;
          break;
      }

      if (debug.main_loop_cnt < 100000) {
        p("loop %6d", debug.main_loop_cnt / 10);
      }
      if (debug.timer_itr_exit_cnt > 1500) {  // 2ms cycleのとき、max 2000cnt
        p("cnt %4d", debug.timer_itr_exit_cnt);
      }
      p("\n\r");
      HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)printf_buffer, strlen(printf_buffer));

      debug.main_loop_cnt = 0;
      debug.pre_yaw_angle = imu.yaw_angle;

      debug.true_cycle_cnt = 0;
      debug.true_fb_total_spin = 0;
      debug.true_out_total_spi = 0;
      debug.uart_rx_itr_cnt = 0;
    }
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

void canRxTimeoutCntCycle(void)
{
  for (int i = 0; i < BOARD_ID_MAX; i++) {
    can_raw.board_rx_timeout_cnt[i]++;
  }
}

// エンコーダ&オムニ角度取得済みかどうか
bool allEncInitialized() { return can_raw.mouse_rx_flag & can_raw.enc_rx_flag[0] & can_raw.enc_rx_flag[1] & can_raw.enc_rx_flag[2] & can_raw.enc_rx_flag[3]; }

// 必要なデータが揃ったらodom系をゼロで初期化
void resetOdomAtEncInitialized()
{
  static bool initialized_flag = false;

  if (initialized_flag == false && allEncInitialized()) {
    for (int i = 0; i < 2; i++) {
      omni.odom[i] = 0;
      omni.pre_odom[i] = 0;
      mouse.floor_odom[i] = 0;
      omni.odom_raw[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
      motor.pre_enc_angle[i] = motor.enc_angle[i];
    }
    initialized_flag = true;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs)
{
  uint8_t RxData[CAN_RX_DATA_SIZE];
  FDCAN_RxHeaderTypeDef RxHeader;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
      Error_Handler();
    }

    parseCanCmd(RxHeader.Identifier, RxData, &can_raw, &sys, &motor, &mouse);
    // 関数のネストを浅くするためにparseCanCmd()の中から移動
    if (RxHeader.Identifier == 0x241) {
      mouseOdometry(&mouse, &imu);
    }
  }
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef * hfdcan) { canTxEmptyInterrupt(hfdcan); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  sys.system_time_ms += (1000 / MAIN_LOOP_CYCLE);
  mouse.integral_loop_cnt++;
  // TIM interrupt is TIM7 only.

  debug.start_time[0] = htim7.Instance->CNT;  // パフォーマンス計測用

  // sys.main_mode設定
  static uint8_t pre_sw_mode, sw_mode;
  pre_sw_mode = sw_mode;
  sw_mode = getModeSwitch();

  if (connection.updated_flag) {
    connection.updated_flag = false;
    memcpy(&ai_cmd, &ai_cmd_buf, sizeof(ai_cmd));
    integ.pre_global_target_position[0] = ai_cmd.global_target_position[0];
    integ.pre_global_target_position[1] = ai_cmd.global_target_position[1];
  }

  canRxTimeoutCntCycle();

  if (sys.error_flag) {
    // 一定回数はリセットを許容する
    if (sys.error_id < 4 && sys.error_info == BLDC_ERROR_OVER_CURRENT && sys.error_resume_cnt < 10) {
      sys.error_flag = 0;
      sys.error_info = 0;
      sys.error_value = 0;

      sys.error_resume_cnt++;

      // しばらくstopに落とす
      sys.stop_flag_request_time = sys.system_time_ms + 3000;

      // OFFコマンドでリセット
      actuator_power_ONOFF(0);
    } else {
      sys.main_mode = MAIN_MODE_ERROR;
      resetLocalSpeedControl(&ai_cmd);
    }
  } else if (sw_mode != pre_sw_mode) {  // reset
    sys.main_mode = MAIN_MODE_NONE;
    resetLocalSpeedControl(&ai_cmd);
  } else {
    sys.main_mode = sw_mode;
  }

  if (sys.system_time_ms < sys.stop_flag_request_time || canRxTimeoutDetection(&can_raw) || !allEncInitialized()) {
    resetLocalSpeedControl(&ai_cmd);
    sys.stop_flag = true;
  } else {
    sys.stop_flag = false;
  }

  // 以後sys.main_modeによる動作切り替え

  debug.start_time[1] = htim7.Instance->CNT;  // パフォーマンス計測用

  yawFilter();
  omniOdometry(&ai_cmd, &motor, &omni, &integ, &connection, &imu);  // 250us
  resetOdomAtEncInitialized();

  //slipDetection();
  debug.start_time[2] = htim7.Instance->CNT;  // パフォーマンス計測用

  debug.true_out_total_spi += output.motor_voltage[0] + output.motor_voltage[1] + output.motor_voltage[2] + output.motor_voltage[3];
  debug.true_fb_total_spin += can_raw.motor_feedback[0] + can_raw.motor_feedback[1] + can_raw.motor_feedback[2] + can_raw.motor_feedback[3];
  debug.true_cycle_cnt++;

  debug.start_time[3] = htim7.Instance->CNT;  // パフォーマンス計測用

  switch (sys.main_mode) {
    case MAIN_MODE_COMBINATION_CONTROL:  // ローカル統合制御あり
    case MAIN_MODE_SPEED_CONTROL_ONLY:   // ローカル統合制御なし
      if (/*connection.connected_ai == false || */ sys.stop_flag) {
        maintask_stop(&output);
      } else {
        maintask_run();
      }
      break;
    case MAIN_MODE_CMD_DEBUG_MODE:  // local test mode, Visionなし前提。
                                    // 相補フィルタなし、
      if (sys.stop_flag) {
        maintask_stop(&output);
      } else {
        maintask_run();
      }
      break;

    case MAIN_MODE_MOTOR_TEST:  // motor test
      motor_test(&sys, &output);
      break;

    case MAIN_MODE_DRIBBLER_TEST:  // dribble test
      dribbler_test(&sys, &output);
      break;

    case MAIN_MODE_KICKER_AUTO_TEST:  // kicker test (auto)
      kicker_test(&sys, &can_raw, false, &output);
      break;

    case MAIN_MODE_KICKER_MANUAL:  // kicker test (manual)
      kicker_test(&sys, &can_raw, true, &output);
      break;

    case MAIN_MODE_MOTOR_CALIBRATION:
      motor_calibration(&sys);
      break;

    case MAIN_MODE_ERROR:  // error
      maintask_stop(&output);
      send_can_error();
      break;

    default:
      maintask_stop(&output);
      break;
  }

  debug.start_time[4] = htim7.Instance->CNT;  // パフォーマンス計測用
  buzzerControl(&can_raw, &sys, &connection, &ai_cmd);

  debug.start_time[5] = htim7.Instance->CNT;  // パフォーマンス計測用
  communicationStateCheck(&connection, &sys, &ai_cmd);

  // interrupt : 500Hz
  static uint16_t cnt_time_50Hz;
  cnt_time_50Hz++;
  if (cnt_time_50Hz >= 10) {
    cnt_time_50Hz = 0;

    debug.print_flag = true;

    actuator_power_ONOFF(1);

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
  }
  debug.start_time[6] = htim7.Instance->CNT;  // パフォーマンス計測用
  debug.timer_itr_exit_cnt = htim7.Instance->CNT;
}

uint8_t getModeSwitch()
{
  return 15 - (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) + (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) << 1) + (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) << 3) + (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) << 2));
}

void yawFilter()
{
  // 静止中に一気にvision角度を合わせるやつ
  static uint32_t yaw_angle_update_cnt = 0;
  imu.yaw_angle_diff_integral += fabs(imu.pre_yaw_angle - imu.yaw_angle);
  yaw_angle_update_cnt++;
  if (yaw_angle_update_cnt > MAIN_LOOP_CYCLE / 2) {  // 2Hz
    yaw_angle_update_cnt = 0;
    if (imu.yaw_angle_diff_integral < 1) {
      // 機体が旋回していないとき
      debug.theta_override_flag = true;

      // visionとの角度差があるときにアプデ
      if (connection.connected_ai && !ai_cmd.vision_lost_flag && getAngleDiff(imu.yaw_angle, ai_cmd.global_vision_theta) > 10) {
        imu.yaw_angle = ai_cmd.global_vision_theta * 180 / M_PI;
      }
    } else {
      debug.theta_override_flag = false;
    }
    imu.yaw_angle_diff_integral = 0;
  }

  imu.pre_yaw_angle_rad = imu.yaw_angle_rad;
  imu.pre_yaw_angle = imu.yaw_angle;

  // vision更新されたときに強制更新するやつ
  if (ai_cmd.vision_lost_flag == false && ai_cmd.pre_vision_lost_flag == true) {
    imu.yaw_angle = ai_cmd.global_vision_theta * 180 / M_PI;
  }
  ai_cmd.pre_vision_lost_flag = ai_cmd.vision_lost_flag;

  ICM20602_read_IMU_data((float)1.0 / MAIN_LOOP_CYCLE, &(imu.yaw_angle));

  if (sys.main_mode == MAIN_MODE_CMD_DEBUG_MODE) {
    // デバッグ用、targetへ補正する
    imu.yaw_angle = imu.yaw_angle - (getAngleDiff(imu.yaw_angle * M_PI / 180.0, ai_cmd.target_theta) * 180.0 / M_PI) * 0.001;  // 0.001 : gain

  } else if (ai_cmd.vision_lost_flag || debug.latency_check_enabled) {
    // VisionLost時は補正しない
    // レイテンシチェック中(一定速度での旋回中)は相補フィルタ切る

  } else {
    imu.yaw_angle = imu.yaw_angle - (getAngleDiff(imu.yaw_angle * M_PI / 180.0, ai_cmd.global_vision_theta) * 180.0 / M_PI) * 0.001;  // 0.001 : gain
  }

  imu.yaw_angle_rad = imu.yaw_angle * M_PI / 180;
}

// 後輪だけ位置制御に使っているせいでタイヤ回転数によるスリップ検出がだいぶ無理がある
void slipDetection(void)
{
  for (int calc_idx = 0; calc_idx < 4; calc_idx++) {
    slip_detect.spin_total[calc_idx] = 0;
    for (int motor_idx = 0; motor_idx < 4; motor_idx++) {
      if (motor_idx != calc_idx) {
        slip_detect.spin_total[calc_idx] += can_raw.motor_feedback_velocity[motor_idx];
      }
    }
  }

  //対角
  slip_detect.diff[0] = slip_detect.spin_total[0] - slip_detect.spin_total[2];
  slip_detect.diff[1] = slip_detect.spin_total[1] - slip_detect.spin_total[3];

  // 左右
  slip_detect.diff[2] = slip_detect.spin_total[0] - slip_detect.spin_total[1];
  slip_detect.diff[3] = slip_detect.spin_total[2] - slip_detect.spin_total[3];
  /* slip_detect.spin_total[0] = can_raw.motor_feedback_velocity[1] + can_raw.motor_feedback_velocity[2] + can_raw.motor_feedback_velocity[3];
  slip_detect.spin_total[1] = can_raw.motor_feedback_velocity[0] + can_raw.motor_feedback_velocity[2] + can_raw.motor_feedback_velocity[3];
  slip_detect.spin_total[2] = can_raw.motor_feedback_velocity[0] + can_raw.motor_feedback_velocity[1] + can_raw.motor_feedback_velocity[3];
  slip_detect.spin_total[3] = can_raw.motor_feedback_velocity[0] + can_raw.motor_feedback_velocity[1] + can_raw.motor_feedback_velocity[2];*/
}

void maintask_run()
{
  // 全部で250us
  local_feedback(&integ, &imu, &sys, &target, &ai_cmd, &omni, &mouse);
  accel_control(&acc_vel, &output, &target, &imu, &omni);
  speed_control(&acc_vel, &output, &target, &imu, &omni);
  output_limit(&output, &debug);

  if (isLatencyCheckModeEnabled(&sys, &debug, &ai_cmd)) {
    debug.rotation_target_theta = getTargetThetaInLatencyCheckMode(&debug, &ai_cmd);
    theta_control(debug.rotation_target_theta, &acc_vel, &output, &imu);
  } else {
    theta_control(ai_cmd.target_theta, &acc_vel, &output, &imu);
  }

  // デバッグモードではstopとvision_lostを無視する
  if (sys.main_mode != MAIN_MODE_CMD_DEBUG_MODE && (ai_cmd.stop_request_flag || ai_cmd.vision_lost_flag)) {
    resetLocalSpeedControl(&ai_cmd);
    omni_move(0.0, 0.0, 0.0, 0.0, &output);
  } else {
    omni_move(output.velocity[0], output.velocity[1], output.omega, OMNI_OUTPUT_LIMIT, &output);
  }

  send_actuator_cmd_run(&ai_cmd, &sys, &can_raw);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  static int32_t uart_rx_cmd_idx = 0;
  uint8_t rx_data_tmp;
  debug.uart_rx_itr_cnt++;

  if (huart->Instance == USART2) {
    rx_data_tmp = uart2_rx_it_buffer;
    HAL_UART_Receive_IT(&huart2, &uart2_rx_it_buffer, 1);

    if (uart_rx_cmd_idx >= 0 && uart_rx_cmd_idx < RX_BUF_SIZE_ETHER) {
      data_from_cm4[uart_rx_cmd_idx] = rx_data_tmp;
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
      parseRxCmd(&connection, &sys, &ai_cmd_buf, data_from_cm4);
      sendRobotInfo(&can_raw, &sys, &imu, &omni, &mouse, &ai_cmd_buf, &connection);
    }
  }

  if (huart->Instance == hlpuart1.Instance) {
    debug.print_idx++;
    HAL_UART_Receive_IT(&hlpuart1, &lpuart1_rx_it_buffer, 1);
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
