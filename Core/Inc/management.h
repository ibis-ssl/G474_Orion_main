/*
 * management.h
 *
 *  Created on: Sep 1, 2019
 *      Author: okada_tech
 */

#ifndef MANAGEMENT_H_
#define MANAGEMENT_H_

#include <math.h>
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

//#define ARM_MATH_CM4
//#include "arm_math.h"

enum {
  MAIN_MODE_COMBINATION_CONTROL = 0,
  MAIN_MODE_SPEED_CONTROL_ONLY,
  MAIN_MODE_CMD_DEBUG_MODE,
  MAIN_MODE_MOTOR_TEST,
  MAIN_MODE_DRIBBLER_TEST,
  MAIN_MODE_KICKER_AUTO_TEST,
  MAIN_MODE_KICKER_MANUAL,
  MAIN_MODE_LATENCY_CHECK,
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

#define LOW_VOLTAGE_LIMIT (22.0)

#define MAIN_LOOP_CYCLE (500)

#define CAN_RX_DATA_SIZE 8
#define CAN_TX_DATA_SIZE 8
#define OMNI_DIAMETER 0.056
#define ROBOT_RADIUS 0.080
#define RX_BUF_SIZE_CM4 (64 + 7)
#define TX_BUF_SIZE_CM4 128

// logging time : 0.5s -> 2s : without vision mode
#define SPEED_LOG_BUF_SIZE (MAIN_LOOP_CYCLE * 2)
#define SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE (10)

#define AI_CMD_VEL_MAX_MPS (7.0)

#define FLAG_SSL_VISION_OK (0x01)
#define FLAG_ENABLE_KEEPER_MODE (0x02)
#define FLAG_STOP_REQUEST (0x04)
#define FLAG_ENABLE_LOCAL_VISION (0x08)
#define FLAG_DRIBBLER_UP (0x10)

enum {
  BOARD_ID_POWER = 0,
  BOARD_ID_MOTOR_RIGHT,
  BOARD_ID_MOTOR_LEFT,
  BOARD_ID_SUB,
  BOARD_ID_MAX,
};

typedef struct
{
  float * buffer;  // float型のデータを格納する配列
  int size;        // バッファのサイズ
  int front;       // データの先頭位置
  int rear;        // データの末尾位置
  int count;       // データの数
} RingBuffer;

typedef struct
{
  float yaw_angle, pre_yaw_angle;
  float yaw_angle_rad, pre_yaw_angle_rad;
  float yaw_angle_diff_integral;
} imu_t;
typedef struct
{
  uint8_t error_no[8];
  float motor_feedback[5];           // rps
  float motor_feedback_velocity[5];  // m/s
  float power_voltage[7];
  float temperature[7];
  float current[5];
  uint8_t ball_detection[4];
  uint32_t board_rx_timeout_cnt[4];
  bool enc_rx_flag[4], mouse_rx_flag;
} can_raw_t;

typedef struct
{
  float enc_angle[5];
  float pre_enc_angle[5];
  float angle_diff[4];
} motor_t;

typedef struct
{
  float global_vel[2];           // m/s 速度指令値の入力
  float local_vel[2];            // m/s 上記とほぼ同じ
  float local_vel_now[2];        // 台形制御指令値
  float local_vel_ff_factor[2];  // 最終指令速度への追従を高めるためのFF項目
  float global_vel_now[2];       // ターゲットグローバル速度
  float global_pos[2];           // 上記で移動するX,Y座標
} target_t;

typedef struct
{
  float vel_error_xy[2];
  float vel_error_scalar, vel_error_rad;
  //float vel_error_real_scalar, vel_error_real_rad;
} accel_vector_t;

typedef struct
{
  float global_vel[2];
  float raw_odom[2];
  float floor_odom[2];
  float odom[2], pre_odom[2];
  int16_t raw[2];
  float raw_diff[2];
  uint16_t quality;
  int integral_loop_cnt, loop_cnt_debug;
  float pre_yaw_angle_rad, diff_yaw_angle_rad;
} mouse_t;

typedef struct
{
  float travel_distance[2];
  float global_odom_diff[2], robot_pos_diff[2];
  float odom[2], pre_odom[2], odom_raw[2];  // フィールド
  float odom_speed[2];                      // フィールド
  float local_odom_speed[2];                // ローカル
  RingBuffer * local_speed_log[2];
  float local_odom_speed_mvf[2];
} omni_t;

typedef struct
{
  RingBuffer * odom_log[2];
  float global_odom_vision_diff[2];     // vision座標を基準にした移動距離(global系)
  float vision_based_position[2];       // Visionによって更新された自己位置(global系)
  float position_diff[2];               // ai_cmdとvision_based_positionの差分(global系)
  float pre_global_target_position[2];  // ai_cmdとvision_based_positionの差分(global系)
  //float move_dist;                      // Visionとtargetが更新されてからの移動量
  //float target_dist_diff;               // Visionが更新された時点での現在地とtargetの距離
  float local_target_diff[2];
} integration_control_t;

typedef struct
{
  volatile float velocity[2];
  volatile float omega;
  volatile float accel[2];
  volatile float motor_voltage[4];
} output_t;

typedef struct
{
  bool connected_ai;
  bool connected_cm4;
  bool already_connected_ai;
  bool updated_flag;
  uint8_t check_pre;
  uint8_t check_ver;
  float cmd_rx_frq;
  uint32_t vision_update_cycle_cnt;
  uint32_t latest_ai_cmd_update_time;
  uint32_t latest_cm4_cmd_update_time;
} connection_t;

typedef struct
{
  bool error_flag, stop_flag;
  uint16_t error_id, error_info;
  float error_value;
  uint32_t error_resume_cnt;  // エラー時の自動復帰回数上限
  uint8_t main_mode;
  uint32_t system_time_ms;
  uint32_t stop_flag_request_time;
  uint16_t kick_state;
  uint32_t sw_data;
} system_t;
typedef struct
{
  volatile uint32_t print_idx;
  volatile uint32_t main_loop_cnt, true_cycle_cnt;
  volatile float out_total_spin, fb_total_spin, pre_yaw_angle;
  volatile float true_out_total_spi, true_fb_total_spin, true_yaw_speed, limited_output;
  volatile bool print_flag, theta_override_flag;
  volatile bool latency_check_enabled;
  volatile int latency_check_seq_cnt;
  volatile float rotation_target_theta;
  volatile uint32_t uart_rx_itr_cnt;
  volatile uint32_t start_time[20], end_time[20], timer_itr_exit_cnt;  //実行パフォーマンス計測用
} debug_t;

typedef union {
  uint8_t buf[TX_BUF_SIZE_CM4];

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

//extern float voltage[6];

#endif /* MANAGEMENT_H_ */
