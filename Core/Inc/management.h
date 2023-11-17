/*
 * managiment.h
 *
 *  Created on: Sep 1, 2019
 *      Author: okada_tech
 */

#ifndef MANAGEMENT_H_
#define MANAGEMENT_H_

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "math.h"
#include "spi.h"
#include "stdio.h"
#include "stm32g4xx_hal.h"
#include "tim.h"
#include "usart.h"
#define ARM_MATH_CM4
#include "actuator.h"
#include "arm_math.h"
#include "can_ibis.h"
#include "icm20602_spi.h"
#include "microsectimer.h"
#include "myatan2.h"
#include "omni_wheel.h"
#include "util.h"
#include "odom.h"

extern float32_t motor_voltage[4];

#define MAIN_LOOP_CYCLE (500)

#define CAN_RX_DATA_SIZE 8
#define CAN_TX_DATA_SIZE 8
#define OMNI_DIAMETER 0.056
#define ROBOT_RADIUS 0.080
#define RX_BUF_SIZE_ETHER 64
#define TX_BUF_SIZE_ETHER 64

#define SPEED_LOG_BUF_SIZE 100

typedef struct
{
  float yaw_angle, pre_yaw_angle;
  float yaw_angle_rad, pre_yaw_angle_rad;
} imu_t;
typedef struct
{
  uint8_t error_no[4];
  float motor_feedback[5];
  float motor_feedback_velocity[5];
  float power_voltage[7];
  float temperature[7];
  float current[5];
  uint8_t ball_detection[4];
} can_raw_t;

typedef struct
{
  float target_theta, global_vision_theta;
  float drible_power;
  float kick_power;
  bool chip_en;
  float local_target_speed[2];
  int global_robot_position[2];
  int global_target_position[2];
  int global_ball_position[2];
  uint8_t allow_local_flags;
  int ball_local_x, ball_local_y, ball_local_radius, ball_local_FPS;
  bool vision_lost_flag, local_vision_en_flag, keeper_mode_en_flag;
} ai_cmd_t;

typedef struct
{
  float enc_angle[5];
  float pre_enc_angle[5];
  float angle_diff[4];
} motor_t;

typedef struct
{
  float position[2];
  float velocity[2];  // m/s
  float acceleration[2];
  float velocity_current[2];
} target_t;

typedef struct
{
  float velocity[2];
  float raw_odom[2];
  float floor_odom[2];
  float odom[2];
  int16_t raw[2];
  float raw_diff[2];
  uint16_t quality;
  int integral_loop_cnt, loop_cnt_debug;
  float pre_yaw_angle_rad, diff_yaw_angle_rad;
} mouse_t;

typedef struct
{
  float travel_distance[2];
  float odom_floor_diff[2], robot_pos_diff[2];
  float odom[2], pre_odom[2], odom_raw[2];
  float odom_speed[2], odom_speed_log[2][SPEED_LOG_BUF_SIZE];
  float odom_speed_log_total[2];
} omni_t;

typedef struct
{
  volatile float velocity[2];
  volatile float omega;
} output_t;

typedef struct
{
  uint8_t connected_ai;
  uint8_t connected_cm4;
  uint32_t check_pre;
  uint32_t check_ver;
  uint32_t cmd_cnt;
  float cmd_rx_frq;
} connection_t;

typedef struct
{
  bool error_flag;
  bool starting_status_flag;
  uint8_t main_mode;
} system_t;

extern imu_t imu;
extern can_raw_t can_raw;
extern ai_cmd_t ai_cmd;
extern target_t target;
extern mouse_t mouse;
extern omni_t omni;
extern output_t output;
extern motor_t motor;
extern connection_t connection;
extern system_t sys;

//extern float32_t voltage[6];

#endif /* MANAGEMENT_H_ */
