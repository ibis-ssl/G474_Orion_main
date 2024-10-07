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

#include "management.h"
#include "robot_packet.h"
//

#include "actuator.h"
#include "ai_comm.h"
#include "buzzer_control.h"
#include "can_ibis.h"
#include "control_theta.h"
#include "error.h"
#include "icm20602_spi.h"
#include "odom.h"
#include "omni_wheel.h"
#include "ring_buffer.h"
#include "robot_control.h"
#include "state_func.h"
#include "stop_state_control.h"
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
bool allEncInitialized();
uint32_t HAL_GetTick(void) { return uwTick; }

// shared with other files
imu_t imu;
can_raw_t can_raw;
target_t target;
mouse_t mouse;
omni_t omni;
output_t output;
motor_t motor;
connection_t connection;
system_t sys;
integ_control_t integ;
accel_vector_t acc_vel;
debug_t debug;
omega_target_t omega_target;
camera_t camera, camera_buf;

RobotCommandV2 cmd_v2, cmd_v2_buf;

UART_HandleTypeDef * huart_xprintf;

#define printf_BUF_SIZE 500
static char printf_buffer[printf_BUF_SIZE];

enum {
  PRINT_IDX_AI_CMD = 0,
  PRINT_IDX_MOTOR,
  PRINT_IDX_DRIBBLER,
  PRINT_IDX_KICKER,
  PRINT_IDX_MOUSE,
  PRINT_IDX_ODOM,
  PRINT_IDX_MOTION,
  PRINT_IDX_VEL,
  PRINT_IDX_LATENCY,
  PRINT_IDX_SYSTEM,
  PRINT_IDX_UART_RAW,
  PRINT_IDX_MAX
};

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
uint8_t data_from_cm4[RX_BUF_SIZE_CM4];
uint8_t tx_data_uart[TX_BUF_SIZE_CM4];
uint8_t uart2_rx_it_buffer = 0, lpuart1_rx_buf = 0;

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

void setTextRed() { p("\e[31m"); }
void setTextYellow() { p("\e[33m"); }
void setTextGreen() { p("\e[32m"); }
//void setTextBlue() { p("\e[34m"); } //見づらいので非推奨
void setTextMagenta() { p("\e[35m"); }
void setTextCyan() { p("\e[36m"); }
void setTextNormal() { p("\e[37m"); }

// たまにLPUARTの割り込みが停止するので自動復帰
void checkAndRestartLPUART_IT(void)
{
  if ((hlpuart1.Instance->CR1 & USART_CR1_RXNEIE) == 0) {
    HAL_UART_Receive_IT(&hlpuart1, &lpuart1_rx_buf, 1);
  }
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
  omni.local_speed_log[2] = initRingBuffer(SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE);

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
  HAL_UART_Receive_IT(&hlpuart1, &lpuart1_rx_buf, 1);

  printf("orion main start %s %s\r\n", __DATE__, __TIME__);

  HAL_ADC_Start_DMA(&hadc5, &sys.sw_adc_raw, 1);

  setHighUartRxLED();
  setHighInterruptLED();
  setHighEventLED();
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

  actuatorPower_ONOFF(0);
  HAL_Delay(20);

  actuator_motor1(0.0);
  actuator_motor2(0.0);
  actuator_motor3(0.0);
  actuator_motor4(0.0);
  actuator_motor5(0.0);

  kicker_charge_start();
  actuator_kicker_cmd_voltage(0.0);
  actuator_power_param(1, 15.0);  // min voltage
  actuator_power_param(2, 35.0);  // max voltage
  actuator_power_param(3, 50.0);  // max can_raw.current
  actuator_power_param(4, 90.0);  // max temp(fet)
  actuator_power_param(5, 90.0);  // max temp(solenoid)

  setLowUartRxLED();
  setLowEventLED();
  setLowInterruptLED();
  actuatorPower_ONOFF(1);

  actuator_buzzer_frq(1046, 50);  //C5
  actuator_buzzer_frq(1174, 50);  //D5
  actuator_buzzer_frq(1046, 50);  //C5
  actuator_buzzer_frq(1174, 50);  //D5
  actuator_buzzer_frq(1318, 50);  //E5
  actuator_buzzer_frq(1396, 50);  //F5

  actuatorPower_ONOFF(1);

  sys.system_time_ms = 1000;  //
  requestStop(&sys, 1000);    // !!注意!! TIM7の割り込みがはじまってから1000ms間停止
  connection.already_connected_ai = false;
  HAL_Delay(100);
  HAL_TIM_Base_Start_IT(&htim7);
  // TIM interrupt is TIM7 only.

  HAL_Delay(500);
  debug.print_idx = PRINT_IDX_VEL;
  unsigned char error_str[100] = {0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    debug.sys_mnt.main_loop_cnt++;
    if (debug.print_flag /* && fabs(omni.local_odom_speed_mvf[0]) > 0.01*/) {
      debug.print_flag = false;

      // 文字列初期化
      printf_buffer[0] = 0;

      p("\e[0m");  //初期化
      if (isLowVoltage(&can_raw)) {
        setTextYellow();
      }
      p("Batt=%3.1f ", getBatteryRemain(&can_raw));
      setTextNormal();

      if (sys.main_mode == MAIN_MODE_ERROR) {
        // 赤
        setTextRed();
        //p(" error : ID %5d / Info %5d / Value %+8.3f ", sys.current_error.id, sys.current_error.info, sys.current_error.value);
        convertErrorDataToStr(sys.current_error.id, sys.current_error.info, error_str);
        p("Err %s %+5.2f ", error_str, sys.current_error.value);
        setTextNormal();
      }
      if (isStopRequested(&sys)) {
        //黄色
        setTextYellow();
      }

      switch (debug.print_idx) {
        case PRINT_IDX_AI_CMD:
          // 通信接続状態表示
          //p("itr %4d %8d %8d ", debug.sys_mnt.uart_rx_itr_cnt, connection.latest_cm4_cmd_update_time, connection.latest_ai_cmd_update_time);

          p("CMD ");
          p("STP%d ", cmd_v2.stop_emergency);

          if (connection.connected_ai) {
            p("\e[32m%3d,%3.0f\e[37m ", cmd_v2.check_counter, connection.ai_cmd_rx_frq);
          } else if (connection.connected_cm4) {
            // 33 : yellow
            p("\e[33m%3d,%3.0f\e[37m ", cmd_v2.check_counter, connection.ai_cmd_rx_frq);
          } else {
            // 31 : red
            p("\e[31m%3d,%3.0f\e[37m ", cmd_v2.check_counter, connection.ai_cmd_rx_frq);
          }

          if (cmd_v2.is_vision_available && cmd_v2.elapsed_time_ms_since_last_vision < 200) {
            setTextGreen();
          } else if (!cmd_v2.stop_emergency) {
            setTextYellow();
          } else {
            setTextRed();
          }
          p("VisionX %+6.2f Y %+6.2f ", cmd_v2.vision_global_pos[0], cmd_v2.vision_global_pos[1]);
          p("theta %+6.1f ", cmd_v2.vision_global_theta * 180 / M_PI);
          p("elt %4d ", cmd_v2.elapsed_time_ms_since_last_vision);
          setTextNormal();
          p("Ltcy %3d ", cmd_v2.latency_time_ms);

          p("TarTheta %+6.2f ", cmd_v2.target_global_theta);
          p("Lmt: Spd %4.2f Omg %4.1f Acc %4.2f / ", cmd_v2.linear_velocity_limit, cmd_v2.angular_velocity_limit, cmd_v2.acceleration_limit);

          p("dri %+4.2f ", cmd_v2.dribble_power);
          if (cmd_v2.lift_dribbler) {
            setTextCyan();
            p("UP ");
          } else {
            setTextMagenta();
            p("DN ");
          }
          if (cmd_v2.enable_chip) {
            setTextCyan();
            p("chip %3.2f ", cmd_v2.kick_power);
          } else {
            setTextMagenta();
            p("stlt %3.2f ", cmd_v2.kick_power);
          }

          if (cmd_v2.prioritize_accurate_acceleration) {
            setTextCyan();
            p("Pri-Acur ");
          } else if (cmd_v2.prioritize_move) {
            setTextMagenta();
            p("Pri-Move ");
          }
          setTextNormal();

          /*********************************************************************************************** */
          switch (cmd_v2.control_mode) {
            case LOCAL_CAMERA_MODE:
              p("CAM ");
              p("BallX %+6.2f Y %+6.2f ", cmd_v2.mode_args.local_camera.ball_pos[0], cmd_v2.mode_args.local_camera.ball_pos[1]);
              p("VelX %+4.2f VelY %+4.2f ", cmd_v2.mode_args.local_camera.ball_vel[0], cmd_v2.mode_args.local_camera.ball_vel[1]);
              p("TarX %+4.2f TarY %+4.2f ", cmd_v2.mode_args.local_camera.target_global_vel[0], cmd_v2.mode_args.local_camera.target_global_vel[1]);

              break;
            case POSITION_TARGET_MODE:
              p("POS ");
              p("TarX %+6.2f Y %+6.2f ", cmd_v2.mode_args.position.target_global_pos[0], cmd_v2.mode_args.position.target_global_pos[1]);
              p("TermSpd %4.1f ", cmd_v2.mode_args.position.terminal_velocity);
              break;
            case SIMPLE_VELOCITY_TARGET_MODE:
              p("SimpleVel ");
              p("VelX %+6.2f %+6.2f ", cmd_v2.mode_args.simple_velocity.target_global_vel[0], cmd_v2.mode_args.simple_velocity.target_global_vel[1]);
              break;

            case VELOCITY_TARGET_WITH_TRAJECTORY_MODE:
              p("VEL ");
              p("VelX %+6.2f Y %+6.2f ", cmd_v2.mode_args.velocity.target_global_vel[0], cmd_v2.mode_args.velocity.target_global_vel[1]);
              p("Tra OrgX %+6.2f Y %+6.2f ", cmd_v2.mode_args.velocity.trajectory_global_origin[0], cmd_v2.mode_args.velocity.trajectory_global_origin[1]);
              p("Angle %+6.2f Cur %+6.2f ", cmd_v2.mode_args.velocity.trajectory_origin_angle, cmd_v2.mode_args.velocity.trajectory_curvature);
              break;
            default:
              p("Unknown Mode ");
              break;
          }
          /*********************************************************************************************** */

          break;
        case PRINT_IDX_MOTOR:  //Motor

          p("MOTOR ");
          p("Spd %+6.1f %+6.1f %+6.1f %+6.1f / ", can_raw.motor_feedback[0], can_raw.motor_feedback[1], can_raw.motor_feedback[2], can_raw.motor_feedback[3]);
          //p("Spd M0=%+6.1f M1=%+6.1f M2=%+6.1f M3=%+6.1f / ", omni.travel_distance[0], omni.travel_distance[1], omni.travel_distance[2], omni.travel_distance[3]);
          p("PwV %5.1f %5.1f %5.1f %5.1f / ", can_raw.power_voltage[0], can_raw.power_voltage[1], can_raw.power_voltage[2], can_raw.power_voltage[3]);
          p("Im %+5.1f %+5.1f %+5.1f %+5.1f / ", can_raw.current[0], can_raw.current[1], can_raw.current[2], can_raw.current[3]);
          const int MOTOR_OVER_HEAT_WARN_THRESH = 60;
          p("Temp ");
          for (int i = 0; i < 4; i++) {
            if (can_raw.temperature[i] > MOTOR_OVER_HEAT_WARN_THRESH) {
              setTextYellow();
            } else {
              setTextNormal();
            }
            p("M%d %3.0f ", i, can_raw.temperature[i]);
          }
          setTextNormal();
          p("Rps ");
          for (int i = 0; i < 4; i++) {
            p("%4.2f ", can_raw.motor_rps[i]);
          }
          break;
        case PRINT_IDX_DRIBBLER:  // Dribblerテスト
          p("DRIBBLER ");
          p("Batt(Sub) %3.1f / ", can_raw.power_voltage[4]);
          p("ball_sensor %d %d / ESC Spd %+5.0f / ", can_raw.ball_detection[0], can_raw.ball_detection[1], can_raw.motor_feedback_velocity[4]);
          //p("local_vision x=%3d y=%3d radius=%3d FPS=%3d ", ai_cmd.ball_local_x, ai_cmd.ball_local_y, ai_cmd.ball_local_radius, ai_cmd.ball_local_FPS);

          break;
        case PRINT_IDX_KICKER:  // Kicker Test
          p("KICKER ");
          p("Cnt %3d ", sys.kick_state);
          p("Batt(Pw) %3.1f ", can_raw.power_voltage[5]);
          if (can_raw.power_voltage[6] > 100) {
            setTextYellow();
          } else {
            setTextNormal();
          }
          p("Cap=%3.0f ", can_raw.power_voltage[6]);
          setTextNormal();
          p("BattC %+6.1f Batt(Sub) %3.1f / ", can_raw.current[4], can_raw.power_voltage[4]);
          const int KICKER_OVER_HEAT_WARN_THRESH = 70;  // power側でエラーは80度
          if (can_raw.temperature[4] > KICKER_OVER_HEAT_WARN_THRESH || can_raw.temperature[5] > KICKER_OVER_HEAT_WARN_THRESH || can_raw.temperature[6] > KICKER_OVER_HEAT_WARN_THRESH) {
            setTextYellow();
          } else {
            setTextNormal();
          }
          p("FET=%5.1f L1=%5.1f L2=%5.1f / ", can_raw.temperature[4], can_raw.temperature[5], can_raw.temperature[6]);
          setTextNormal();
          p("ball_sensor %d %d / ESC Spd %+5.0f ", can_raw.ball_detection[0], can_raw.ball_detection[1], can_raw.motor_feedback_velocity[4]);

          break;
        case PRINT_IDX_MOUSE:  // Mouse odom
          p("MOUSE ");
          p("%d / enc all%d %d %d %d %d / ", can_raw.rx_stat.mouse_flag, allEncInitialized(&can_raw), can_raw.rx_stat.enc_flag[0], can_raw.rx_stat.enc_flag[1], can_raw.rx_stat.enc_flag[2],
            can_raw.rx_stat.enc_flag[3]);
          //p("raw_odom X %+8.3f Y %+8.3f ", mouse.raw_odom[0] * 1000, mouse.raw_odom[1] * 1000);
          //p("mouse floor X %+8.3f Y %+8.3f ", mouse.floor_odom[0] * 1000, mouse.floor_odom[1] * 1000);
          p("omni-odom X %+8.1f ,Y %+8.1f ", omni.odom[0] * 1000, omni.odom[1] * 1000);
          p("mouse X %+8.2f Y %+8.2f ", mouse.odom[0] * 1000, mouse.odom[1] * 1000);
          p("Error X %+8.2f Y %+8.2f ", (mouse.odom[0] - omni.odom[0]) * 1000, (mouse.odom[1] - omni.odom[1]) * 1000);
          p("diff X %+8.2f Y %+8.2f ", mouse.raw_diff[0] * 1000, mouse.raw_diff[1] * 1000);
          p("MsVel X %+8.2f Y %+8.2f ", mouse.global_vel[0], mouse.global_vel[1]);
          //p("mouseRaw X %+8.1f Y %+8.1f ", mouse.raw[0], mouse.raw[1]);
          if (mouse.quality < 100) {
            setTextRed();
          } else if (mouse.quality < 150) {
            setTextYellow();
          } else {
            setTextGreen();
          }
          p("/ raw X %+4d Y %+4d Q %6d", mouse.raw[0], mouse.raw[1], mouse.quality);

          break;
        case PRINT_IDX_ODOM:
          p("ODOM ");

          if (cmd_v2.is_vision_available && cmd_v2.elapsed_time_ms_since_last_vision < 200) {
            setTextGreen();
          } else if (!cmd_v2.stop_emergency) {
            setTextYellow();
          } else {
            setTextRed();
          }

          p("VisionX %+6.2f Y %+6.2f ", cmd_v2.vision_global_pos[0], cmd_v2.vision_global_pos[1]);
          p("theta %+6.1f ", cmd_v2.vision_global_theta * 180 / M_PI);
          p("elt %4d ", cmd_v2.elapsed_time_ms_since_last_vision);
          setTextNormal();

          p("imu %+5.1f ", imu.yaw_deg);
          p("Dt %4d LtCy %4d ", connection.ai_cmd_delta_time, integ.latency_cycle);
          p("integ.govd %+7.2f %+7.2f ", integ.global_odom_vision_diff[0] * 1000, integ.global_odom_vision_diff[1] * 1000);
          p("integ.vbp %+5.2f %+5.2f ", integ.vision_based_position[0], integ.vision_based_position[1]);
          p("omni.gos X %+8.1f ,Y %+8.1f. ", omni.global_odom_speed[0] * 1000, omni.global_odom_speed[1] * 1000);
          p("omgD %+6.3f ", getAngleDiff(imu.yaw_rad, imu.pre_yaw_rad) * 4000);
          p("omg %+6.2f ", output.omega);
          break;
        case PRINT_IDX_MOTION:
          p("MOTION ");
          p("theta %+6.1f ", imu.yaw_deg);

          //p("omg %+3.0f out %+5.2f, %+5.2f ", output.omega, output.velocity[0], output.velocity[1]);
          //p("ENC angle %+6.3f %+6.3f %+6.3f %+6.3f ", motor.enc_angle[0], motor.enc_angle[1], motor.enc_angle[2], motor.enc_angle[3]);
          //p("odomL X %+8.3f, Y %+8.3f, ", omni.odom[0], omni.odom[1]);
          //p("omni-GV X %+8.1f ,Y %+8.1f. ", omni.global_odom_speed[0] * 1000, omni.global_odom_speed[1] * 1000);
          if (cmd_v2.is_vision_available) {
            setTextCyan();
          } else {
            setTextYellow();
          }
          p("VisionX %+6.2f Y %+6.2f ", cmd_v2.vision_global_pos[0], cmd_v2.vision_global_pos[1]);
          //p("integ.govd %+7.2f %+7.2f ", integ.global_odom_vision_diff[0] * 1000, integ.global_odom_vision_diff[1] * 1000);
          //p("integ.vbp %+5.2f %+5.2f ", integ.vision_based_position[0], integ.vision_based_position[1]);
          p("diffG %+6.3f %+6.3f ", integ.position_diff[0], integ.position_diff[1]);
          p("dist %+6.3f ", target.pos_ctrl.target_pos_dist_scalar);
          //p("Global-VO %+6.3f Y %+6.3f ", target.global_vel[0], target.global_vel[1]);
          //p("angle %+5.2f ", target.pos_ctrl.target_vel_angle);

          //p("GlobalVel %+6.3f Y %+6.3f ", target.pos_ctrl.global_odom_speed[0], target.pos_ctrl.global_odom_speed[1]);
          p("tarScV %+5.2f ", target.pos_ctrl.target_scalar_vel);
          if (target.pos_ctrl.target_scalar_vel > target.pos_ctrl.current_speed_crd[0]) {
            setTextCyan();
          } else {
            setTextMagenta();
          }
          p("CrSpdCrd %+5.2f,%+5.2f ", target.pos_ctrl.current_speed_crd[0], target.pos_ctrl.current_speed_crd[1]);
          setTextNormal();
          p("accCrd X %+6.2f, Y %+6.2f ", target.pos_ctrl.target_crd_acc[0], target.pos_ctrl.target_crd_acc[1]);
          p("accGl X %+6.2f, Y %+6.2f ", target.pos_ctrl.global_acc[0], target.pos_ctrl.global_acc[1]);
          p("%d ", target.pos_ctrl.to_stop_mode_flag);
          //p("accLc X %+8.2f, Y %+8.2f, ", output.accel[0], output.accel[1]);
          //p("local-VO %+6.3f Y %+6.3f ", output.velocity[0], output.velocity[1]);

          //p("MsVel X %+8.4f Y %+8.4f ", mouse.global_vel[0], mouse.global_vel[1]);
          //p("AI X %+4.1f Y %+4.1f ", ai_cmd.local_target_speed[0], ai_cmd.local_target_speed[1]);
          //p("integ-diff %+6.3f %+6.3f ", integ.local_target_diff[0], integ.local_target_diff[1]);
          //p("tar-pos X %+8.1f, Y %+8.1f ", target.global_vel_now[0], target.global_vel_now[1]);
          //p("cmd-vel %+5.2f, %+5.2f, ", target.global_vel[0], target.global_vel[1]);
          //p("vel-now %+6.3f, %+6.3f, ", target.local_vel_now[0], target.local_vel_now[1]);
          //p("vel-diff X %+8.2f, Y %+8.2f, ", acc_vel.vel_error_xy[0] * 1000, acc_vel.vel_error_xy[1] * 1000);
          //p("rad %+8.2f, scalar %+8.2f, ", acc_vel.vel_error_rad * 180 / M_PI, acc_vel.vel_error_scalar * 1000);
          p("out-vel %+5.1f, %+5.1f ", output.velocity[0], output.velocity[1]);
          //p("acc sca %7.4f rad %5.2f ", acc_vel.vel_error_scalar, acc_vel.vel_error_rad);
          p("real-vel X %+7.2f, Y %+7.2f, 2 %+7.2f,", omni.local_odom_speed_mvf[0], omni.local_odom_speed_mvf[1], omni.local_odom_speed_mvf[2]);

          //p("vel-diff %+8.3f, %+8.3f, ", target.local_vel_now[0] - omni.local_odom_speed_mvf[0], target.local_vel_now[1] - omni.local_odom_speed_mvf[1]);

          break;
        case PRINT_IDX_VEL:
          p("VEL ");
          //p("theta %+6.1f ", imu.yaw_deg);
          //p("rawVel X %+8.3f, Y %+8.3f, ", omni.local_raw_odom_vel[0], omni.local_raw_odom_vel[1]);
          //p("rawGbVel X %+8.3f, Y %+8.3f ", omni.global_raw_odom_vel[0], omni.global_raw_odom_vel[1]);
          //p("rawGbOdom X %+8.3f, Y %+8.3f, ", omni.global_raw_odom[0], omni.global_raw_odom[1]);
          //p("odom X %+8.3f, Y %+8.3f, ", omni.odom[0], omni.odom[1]);
          //p("tar X %+8.3f, Y %+8.3f, ", target.global_pos[0], target.global_pos[1]);
          //p("offset X %+8.3f, Y %+8.3f, ", omni.offset_dist[0], omni.offset_dist[1]);
          //p("odomSpd X %+8.3f, Y %+8.3f, ", omni.global_odom_speed[0], omni.global_odom_speed[1]);
          //p("real-vel X %+8.3f, Y %+8.3f, X2 %+8.3f,", omni.local_odom_speed_mvf[0], omni.local_odom_speed_mvf[1], omni.local_odom_speed_mvf[2]);
          //p("out-vel X %+5.1f, Y %+5.1f W %+5.1f ", output.velocity[0], output.velocity[1], output.omega);
          //p("M0 %+5.2f M1 %+5.2f M2 %+5.2f M3 %+5.2f ", output.motor_voltage[0], output.motor_voltage[1], output.motor_voltage[2], output.motor_voltage[3]);
          p("Err X%+5.2f Y%+5.2f Sc%+5.2f ", acc_vel.vel_error_xy[0], acc_vel.vel_error_xy[1], acc_vel.vel_error_scalar);
          p("accLc X %+8.2f, Y %+8.2f, ", output.accel[0], output.accel[1]);
          //p("TarLocal %+5.1f %+5.1f TarGlobalN %+5.1f %+5.1f TarLocalN %+5.1f %+5.1f ", target.local_vel[0], target.local_vel[1], target.global_vel_now[0], target.global_vel_now[1],target.local_vel_now[0], target.local_vel_now[1]);
          p("Diff X %+5.3f, Y %+5.3f, ", omni.robot_pos_diff[0], omni.robot_pos_diff[1]);  // x150は出力ゲイン
          p("FF-N %+5.1f FF-T %+5.1f ", target.local_vel_ff_factor[0], target.local_vel_ff_factor[1]);
          p("vel-now %+6.3f, %+6.3f, ", target.local_vel_now[0], target.local_vel_now[1]);
          p("real-vel X %+7.2f, Y %+7.2f, 2 %+7.2f,", omni.local_odom_speed_mvf[0], omni.local_odom_speed_mvf[1], omni.local_odom_speed_mvf[2]);

          break;
        case PRINT_IDX_LATENCY:
          p("LATENCY ");
          p("setting : %3d / ", cmd_v2.latency_time_ms);
          p("EN %d cnt %4d target %+5.2f diff %+5.2f / ", debug.latency_check.enabled, debug.latency_check.seq_cnt, debug.latency_check.rotation_target_theta,
            getAngleDiff(debug.latency_check.rotation_target_theta, imu.yaw_rad));

          p("CAM ");
          p("x %+4d y%+4d rad %4d fps %3d ", camera.pos_xy[0], camera.pos_xy[1], camera.radius, camera.fps);

          break;
        case PRINT_IDX_SYSTEM:
          p("SYSTEM TIME ");
          for (int i = 0; i < 7; i++) {
            if (debug.sys_mnt.tim_cnt_max[i] < debug.sys_mnt.tim_cnt_now[i]) {
              debug.sys_mnt.tim_cnt_max[i] = debug.sys_mnt.tim_cnt_now[i];
            }
            p("%4d %4d / ", debug.sys_mnt.tim_cnt_now[i], debug.sys_mnt.tim_cnt_max[i]);
          }
          p("PW%4d ", can_raw.rx_stat.timeout_cnt[BOARD_ID_POWER]);
          p("RD%4d ", can_raw.rx_stat.timeout_cnt[BOARD_ID_MOTOR_RIGHT]);
          p("LD%4d ", can_raw.rx_stat.timeout_cnt[BOARD_ID_MOTOR_LEFT]);
          p("SB%4d ", can_raw.rx_stat.timeout_cnt[BOARD_ID_SUB]);
          p("MotorV ");
          for (int i = 0; i < 4; i++) {
            p("%+6.2f ", output.motor_voltage[i]);
          }
          p("SW ADC %4d ", sys.sw_adc_raw);
          convertErrorDataToStr(sys.latest_error.id, sys.latest_error.info, error_str);
          p("ErrLatest %s %+5.2f Resume %2d", error_str, sys.latest_error.value, sys.current_error.resume_cnt);
          break;
        case PRINT_IDX_UART_RAW:
          p("UART RAW ");
          setTextNormal();
          for (int i = 0; i < 64; i++) {
            p("0x%02x ", data_from_cm4[i]);
          }
          setTextCyan();
          for (int i = 64; i < 64 + 8; i++) {
            p("0x%02x ", data_from_cm4[i]);
          }
          setTextMagenta();
          p(" ck 0x%2x , error %4d", connection.check_cnt, connection.check_sum_error_cnt);
          break;
        default:
          debug.print_idx = 0;
          break;
      }

      if (debug.sys_mnt.main_loop_cnt < 100000) {
        p("loop %6d", debug.sys_mnt.main_loop_cnt / 10);
      }
      if (debug.sys_mnt.timer_itr_exit_cnt > 1500) {  // 2ms cycleのとき、max 2000cnt
        p("cnt %4d", debug.sys_mnt.timer_itr_exit_cnt);
      }
      p("\n");
      HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)printf_buffer, strlen(printf_buffer));

      debug.sys_mnt.main_loop_cnt = 0;
      debug.sys_mnt.uart_rx_itr_cnt = 0;

      // whileで毎回呼ぶ必要はないのでprintに併せて呼ぶ
      checkAndRestartLPUART_IT();
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

// 必要なデータが揃ったらodom系をゼロで初期化
void resetOdomAtEncInitialized()
{
  static bool initialized_flag = false;

  if (initialized_flag == false && allEncInitialized(&can_raw)) {
    for (int i = 0; i < 2; i++) {
      omni.odom[i] = 0;
      omni.pre_odom[i] = 0;
      mouse.floor_odom[i] = 0;
      omni.local_raw_odom_vel[i] = 0;
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
      mouseOdometryUpdate(&mouse, &imu);
    }
  }
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef * hfdcan) { canTxEmptyInterrupt(hfdcan); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  sys.system_time_ms += (1000 / MAIN_LOOP_CYCLE);
  // TIM interrupt is TIM7 only.

  debug.sys_mnt.tim_cnt_now[0] = htim7.Instance->CNT;  // パフォーマンス計測用

  // sys.main_mode設定
  static uint8_t sw_mode;
  sw_mode = getModeSwitch();

  // 関数化したほうがええかも
  if (connection.updated_flag) {
    connection.updated_flag = false;
    memcpy(&cmd_v2, &cmd_v2_buf, sizeof(RobotCommandV2));
    memcpy(&camera, &camera_buf, sizeof(camera_t));
  }
  commStateCheck(&connection, &sys, &cmd_v2);

  canRxTimeoutCntCycle(&can_raw);

  sys.can_timeout = canRxTimeoutDetection(&can_raw);
  sys.enc_initialized = allEncInitialized(&can_raw);

  stopStateControl(&sys, sw_mode);

  // 以後sys.main_modeによる動作切り替え

  debug.sys_mnt.tim_cnt_now[1] = htim7.Instance->CNT;  // パフォーマンス計測用

  yawFilter(&sys, &debug, &imu, &cmd_v2, &connection);
  omniOdometryUpdate(&motor, &omni, &imu);  // 250us
  inntegOdomUpdate(&cmd_v2, &omni, &integ, &connection, &imu, &sys);

  resetOdomAtEncInitialized();

  debug.sys_mnt.tim_cnt_now[2] = htim7.Instance->CNT;  // パフォーマンス計測用

  debug.sys_mnt.tim_cnt_now[3] = htim7.Instance->CNT;  // パフォーマンス計測用

  switch (sys.main_mode) {
    case MAIN_MODE_FULL_AI_CONTROL:  // ローカル統合制御あり
    case MAIN_MODE_MANUAL_CONTROL:   // ローカル統合制御なし
      maintaskRun(&sys, &cmd_v2, &imu, &acc_vel, &integ, &target, &omni, &mouse, &debug, &output, &can_raw, &omega_target);
      break;
    case MAIN_MODE_CMD_DEBUG_MODE:  // local test mode, Visionなし前提。
                                    // 相補フィルタなし、
      maintaskRun(&sys, &cmd_v2, &imu, &acc_vel, &integ, &target, &omni, &mouse, &debug, &output, &can_raw, &omega_target);
      break;

    case MAIN_MODE_MOTOR_TEST:  // motor test
      motorTest(&sys, &output, &omni);
      break;

    case MAIN_MODE_DRIBBLER_TEST:  // dribble test
      dribblerTest(&sys, &output, &can_raw);
      break;

    case MAIN_MODE_KICKER_AUTO_TEST:  // kicker test (auto)
      kickerTest(&sys, &can_raw, false, &output);
      break;

    case MAIN_MODE_KICKER_MANUAL:  // kicker test (manual)
      kickerTest(&sys, &can_raw, true, &output);
      break;

    case MAIN_MODE_LATENCY_CHECK:
      latencyCheck(&sys, &debug, &cmd_v2, &output, &imu);
      break;

    case MAIN_MODE_MOTOR_CALIBRATION:
      motorCalibration(&sys);
      break;

    case MAIN_MODE_ERROR:  // error
      manualPowerReset(&sys);
      maintaskStop(&output);
      sendCanError();
      break;

    default:
      maintaskStop(&output);
      break;
  }

  debug.sys_mnt.tim_cnt_now[4] = htim7.Instance->CNT;  // パフォーマンス計測用
  buzzerControl(&can_raw, &sys, &connection, &cmd_v2);

  debug.sys_mnt.tim_cnt_now[5] = htim7.Instance->CNT;  // パフォーマンス計測用

  // interrupt : 500Hz
  static uint16_t print_cycle_cnt;
  print_cycle_cnt++;
  if (print_cycle_cnt >= (MAIN_LOOP_CYCLE / PRINT_LOOP_CYCLE)) {
    print_cycle_cnt = 0;

    debug.print_flag = true;

    if (sys.main_mode == MAIN_MODE_ERROR) {
      actuatorPower_ONOFF(0);
    } else {
      actuatorPower_ONOFF(1);
    }

    toggleInterruptLED();
  }
  debug.sys_mnt.tim_cnt_now[6] = htim7.Instance->CNT;  // パフォーマンス計測用
  debug.sys_mnt.timer_itr_exit_cnt = htim7.Instance->CNT;
}

uint8_t getModeSwitch()
{
  return 15 - (HAL_GPIO_ReadPin(DIP_0_GPIO_Port, DIP_0_Pin) + (HAL_GPIO_ReadPin(DIP_1_GPIO_Port, DIP_1_Pin) << 1) + (HAL_GPIO_ReadPin(DIP_2_GPIO_Port, DIP_2_Pin) << 3) +
               (HAL_GPIO_ReadPin(DIP_3_GPIO_Port, DIP_3_Pin) << 2));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  static int32_t uart_rx_cmd_idx = -1;
  uint8_t rx_data_tmp;
  RobotCommandSerializedV2 cmd_data_v2;

  debug.sys_mnt.uart_rx_itr_cnt++;

  if (huart->Instance == USART2) {
    rx_data_tmp = uart2_rx_it_buffer;
    HAL_UART_Receive_IT(&huart2, &uart2_rx_it_buffer, 1);

    if (uart_rx_cmd_idx >= 0 && uart_rx_cmd_idx < RX_BUF_SIZE_CM4) {
      data_from_cm4[uart_rx_cmd_idx] = rx_data_tmp;
    }

    // look up header byte
    if (uart_rx_cmd_idx == -1 && rx_data_tmp == 254) {
      data_from_cm4[0] = rx_data_tmp;
      uart_rx_cmd_idx++;
    }

    // data byte
    if (uart_rx_cmd_idx != -1 && uart_rx_cmd_idx < RX_BUF_SIZE_CM4) {
      uart_rx_cmd_idx++;
    }

    // end
    if (uart_rx_cmd_idx == RX_BUF_SIZE_CM4) {
      // UARTバスを送受信で同時に使えないので、受信完了してから送信開始
      sendRobotInfo(&can_raw, &sys, &imu, &omni, &mouse, &cmd_v2, &connection, &integ, &output, &target);

      uart_rx_cmd_idx = -1;
      memcpy(&cmd_data_v2, data_from_cm4, sizeof(cmd_data_v2));
      cmd_v2_buf = RobotCommandSerializedV2_deserialize(&cmd_data_v2);
      camera_buf = parseCameraPacket(&(data_from_cm4[64]));
      if (checkCM4CmdCheckSun(&connection, data_from_cm4)) {
        updateCM4CmdTimeStamp(&connection, &sys);
      }
    }
  }

  if (huart->Instance == hlpuart1.Instance) {
    switch (lpuart1_rx_buf) {
      case 'v':
        debug.print_idx = PRINT_IDX_VEL;
        break;
      case 'w':
        debug.print_idx = PRINT_IDX_MOTOR;
        break;
      case 'm':
        debug.print_idx = PRINT_IDX_MOUSE;
        break;
      case 'p':
        debug.print_idx = PRINT_IDX_MOTION;
        break;
      case 's':
        debug.print_idx = PRINT_IDX_SYSTEM;
        break;
      case 'd':
        debug.print_idx = PRINT_IDX_DRIBBLER;
        break;
      case 'o':
        debug.print_idx = PRINT_IDX_ODOM;
        break;
      case 'l':
        debug.print_idx = PRINT_IDX_LATENCY;
        break;
      case 'u':
        debug.print_idx = PRINT_IDX_UART_RAW;
        break;
      case 0x7f:  // del
        debug.print_idx = 0;
        break;
      case '\n':
      case '\r':
        debug.print_idx++;
        break;
      default:
        debug.print_idx--;
        break;
    }
    if (debug.print_idx < 0) {
      debug.print_idx = PRINT_IDX_MAX - 1;
    }
    HAL_UART_Receive_IT(&hlpuart1, &lpuart1_rx_buf, 1);
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
    setHighEventLED();
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
