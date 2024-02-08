/*
 * managiment.h
 *
 *  Created on: Sep 1, 2019
 *      Author: okada_tech
 */

#ifndef MANAGEMENT_H_
#define MANAGEMENT_H_

#include <stdbool.h>
#include <stdint.h>
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
#include "odom.h"
#include "omni_wheel.h"
#include "ring_buffer.h"
#include "util.h"

extern float32_t motor_voltage[4];

#define MAIN_LOOP_CYCLE (500)

#define CAN_RX_DATA_SIZE 8
#define CAN_TX_DATA_SIZE 8
#define OMNI_DIAMETER 0.056
#define ROBOT_RADIUS 0.080
#define RX_BUF_SIZE_ETHER 64
#define TX_BUF_SIZE_ETHER 128

// logging time : 0.5s -> 2s
#define SPEED_LOG_BUF_SIZE (MAIN_LOOP_CYCLE * 2)

typedef struct
{
  float yaw_angle, pre_yaw_angle;
  float yaw_angle_rad, pre_yaw_angle_rad;
} imu_t;
typedef struct
{
  uint8_t error_no[8];
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
  float global_robot_position[2];
  float global_target_position[2];
  float global_ball_position[2];
  uint8_t allow_local_flags;
  int ball_local_x, ball_local_y, ball_local_radius, ball_local_FPS;
  bool vision_lost_flag, local_vision_en_flag, keeper_mode_en_flag, stop_request_flag, dribbler_up_flag;
  uint32_t latency_time_ms;
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
  float local_position[2];
  float velocity[2];        // m/s
  float local_velocity[2];  // m/s
  float acceleration[2];
  float local_velocity_current[2];
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
  float odom_speed[2];
  float local_odom_speed[2];
} omni_t;

typedef struct
{
  RingBuffer * odom_log[2];
  float global_odom_vision_diff[2];  // vision座標を基準にした移動距離(global系)
  float vision_based_position[2];    // Visionによって更新された自己位置
  float position_diff[2];            // ai_cmdとvision_based_positionの差分
  float pre_global_target_position[2];
  float guess_target_speed[2];  // targetSpeed + 位置制御の場合、使わんかも
  float move_dist;              // Visionとtargetが更新されてからの移動量
  float targed_dist_diff;       // Visionが更新された時点での現在地とtargetの距離
  float local_target_diff[2];
} integration_control_t;

typedef struct
{
  volatile float velocity[2];
  volatile float local_velocity_current[2];
  volatile float omega;
  volatile float accel_limit[2];
} output_t;

typedef struct
{
  bool connected_ai;
  bool connected_cm4;
  uint8_t check_pre;
  uint8_t check_ver;
  float cmd_rx_frq;
  uint32_t vision_update_cycle_cnt;
  uint32_t pre_vision_update_cycle_cnt;
  uint32_t latest_ai_cmd_update_time;
  uint32_t latest_cm4_cmd_update_time;
} connection_t;

typedef struct
{
  bool error_flag, stop_flag;
  uint16_t error_id, error_info;
  float error_value;
  uint8_t main_mode;
  uint32_t system_time_ms;
  uint32_t stop_flag_request_time;
} system_t;

typedef union {
  uint8_t buf[TX_BUF_SIZE_ETHER];

  struct
  {
    uint8_t head[2];
    uint8_t counter, return_counter;  //4

    uint8_t kick_state;
    uint8_t temperature[7];  //12

    uint8_t error_info[8];  //20
    int8_t motor_current[4];
    uint8_t ball_detection[4];  //28

    float yaw_angle, diff_angle;   //36
    float odom[2], odom_speed[2];  //
    float voltage[2];              //
  } data;
} tx_msg_t;

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
extern integration_control_t integ;

//extern float32_t voltage[6];

#endif /* MANAGEMENT_H_ */
