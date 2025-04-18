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
  MAIN_MODE_FULL_AI_CONTROL = 0,
  MAIN_MODE_MANUAL_CONTROL,
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

#define LOW_VOLTAGE_LIMIT (22.5)

#define MAIN_LOOP_CYCLE (500)
#define PRINT_LOOP_CYCLE (50)
#define PRINT_TUI_CYCLE (10)

#define CAN_RX_DATA_SIZE 8
#define CAN_TX_DATA_SIZE 8
#define OMNI_DIAMETER 0.056
#define ROBOT_RADIUS 0.080
#define RX_BUF_SIZE_CM4 (64 + 8)
#define TX_BUF_SIZE_CM4 128

// logging time : 0.5s -> 2s : without vision mode
#define SPEED_LOG_BUF_SIZE (MAIN_LOOP_CYCLE * 2)
#define SPEED_MOVING_AVERAGE_FILTER_BUF_SIZE (3)

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
} ring_buffer_t;

typedef struct
{
  float yaw_deg, pre_yaw_deg;
  float yaw_rad, pre_yaw_rad;
  float yaw_deg_diff_integral;
  bool theta_override_flag;
} imu_t;
typedef struct
{
  uint8_t error_no[8];
  float motor_feedback[5];  // rps
  float power_voltage[7];
  float temp_motor[4], temp_fet, temp_coil[2], temp_driver[4];
  float current[5];
  uint8_t ball_detection[4];
  float motor_param_rps[4];
  struct
  {
    uint32_t timeout_cnt[5];  // omni + dribbler
    bool enc_flag[5];         // omni + dribbler
    bool mouse_flag;
  } rx_stat;
} can_raw_t;

typedef struct
{
  float angle_rad[5];  // これだけ回転方向の定義が逆なので注意
  float pre_angle_rad[5];
  float angle_diff[5];
  float rps[5];
} motor_t;

typedef struct
{
  float vel_error_xy[2];
  float vel_error_scalar, vel_error_rad;
  float accel_target;
  float accel[2];
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
} mouse_t;

typedef struct
{
  float travel_distance[4];
  float global_odom_diff[2], robot_pos_diff[2];
  float odom[2], pre_odom[2], local_raw_odom_vel[2];
  float offset_dist[2];
  float global_raw_odom[3];
  float global_odom_speed[2];  // フィールド
  float local_odom_speed[3];   // ローカル､前輪2輪によるものもあるので3つ
  ring_buffer_t * local_speed_log[3];
  float local_odom_speed_mvf[3];  // ローカル､移動平均｡前輪2輪によるものもあるので3つ
  float global_raw_odom_vel[2];
} omni_t;

typedef struct
{
  ring_buffer_t * odom_log[2];
  float global_odom_vision_diff[2];     // vision座標を基準にした移動距離(global系)
  float vision_based_position[2];       // Visionによって更新された自己位置(global系)
  float position_diff[2];               // ai_cmdとvision_based_positionの差分(global系)
  float pre_global_target_position[2];  // ai_cmdとvision_based_positionの差分(global系)
  float local_target_diff[2];
  int latency_cycle;
} integ_control_t;

typedef struct
{
  float current_tar_rps, pre_tar_rps, angle_rad;
  float diff, real_rps;
  float tar_aps_acc;
  int weak_flag;
} omni_angle_t;
typedef struct
{
  float global_vel[2];      // m/s 速度指令値の入力
  float local_vel[2];       // m/s 上記とほぼ同じ
  float local_vel_now[2];   // 台形制御指令値
  float global_vel_now[2];  // ターゲットグローバル速度
  float global_pos[2];      // 上記で移動するX,Y座標

  float yaw_rps, yaw_rps_drag;

  /*struct
  {
    float target_vel_angle;
    float global_odom_speed[2];
    float current_speed_crd[2];
    float target_crd_acc[2];
    float global_acc[2];
    float target_scalar_vel, target_pos_dist_scalar;
    bool to_stop_mode_flag;
  } pos_ctrl;*/
  omni_angle_t omni_angle[4];
  float omni_angle_kp, omni_angle_kd;
  int weak_flag;
} target_t;

typedef struct
{
  float velocity[2];
  float omega;
  float motor_voltage[4];
} output_t;

typedef struct
{
  bool connected_ai;
  bool connected_cm4;
  bool already_connected_ai;
  bool updated_flag;
  bool camera_updated_flag;
  float ai_cmd_rx_frq;
  int ai_cmd_rx_cnt;
  uint32_t latest_ai_cmd_update_time;
  uint32_t latest_cm4_cmd_update_time;
  int32_t ai_cmd_delta_time;
  uint8_t check_cnt;
  uint32_t check_sum_error_cnt;
} connection_t;

typedef struct
{
  uint16_t id, info;
  float value;
  uint32_t resume_cnt;  // エラー時の自動復帰回数上限
} error_t;

// 機体の挙動に影響するもの
typedef struct
{
  bool error_flag, stop_flag, can_timeout, enc_initialized;
  error_t current_error, latest_error;
  uint8_t main_mode;
  uint32_t system_time_ms;
  uint32_t stop_flag_request_time;
  uint16_t kick_state;
  uint32_t sw_adc_raw;
} system_t;

// 機体の挙動に関係ないもの
typedef struct
{
  volatile int32_t print_idx;
  volatile bool print_flag;
  volatile uint32_t print_cycle;

  struct
  {
    volatile bool enabled;
    volatile int seq_cnt;
    volatile float rotation_target_theta;
  } latency_check;

  struct
  {
    volatile uint32_t tim_cnt_now[20], tim_cnt_max[20], timer_itr_exit_cnt;  //実行パフォーマンス計測用
    volatile uint32_t main_loop_cnt;
    volatile uint32_t uart_rx_itr_cnt;
  } sys_mnt;

} debug_t;

typedef struct
{
  int16_t pos_xy[2], radius;
  uint8_t fps;
  bool is_detected, is_connected;
  uint32_t latest_local_cam_update_time;
} camera_t;

//extern float voltage[6];

#endif /* MANAGEMENT_H_ */
