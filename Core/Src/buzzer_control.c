#include "buzzer_control.h"

#include "actuator.h"
#include "can_ibis.h"
#include "management.h"
#include "stdbool.h"
#include "stdint.h"

inline static bool isLowVoltage(can_raw_t * can_raw)
{
  if (can_raw->power_voltage[5] != 0.0) {
    if (can_raw->power_voltage[5] < LOW_VOLTAGE_LIMIT) {
      return true;
    }
  }
  return false;
}

inline static bool isVisionLost(system_t * sys, connection_t * con, ai_cmd_t * ai_cmd)
{
  if (con->connected_ai) {
    if (ai_cmd->vision_lost_flag) {
      if (sys->main_mode != MAIN_MODE_CMD_DEBUG_MODE) {
        return true;
      }
    }
  }
  return false;
}

inline void buzzerControl(can_raw_t * can_raw, system_t * sys, connection_t * con, ai_cmd_t * ai_cmd)
{
  static bool buzzer_state = false;
  static uint32_t buzzer_cnt = 0;
  static float buzzer_frq_offset__gain = 1.0;
  // 電圧受信できてない時に低電圧エラー鳴るとウザいので消す
  buzzer_cnt++;
  if (isLowVoltage(can_raw)) {  // 低電圧時
    if (buzzer_cnt > 100) {
      buzzer_cnt = 0;
      if (buzzer_state == false) {
        buzzer_state = true;
        actuator_buzzer_frq_on(2000);
      } else {
        buzzer_state = false;
        actuator_buzzer_off();
      }
    }
  } else if (sys->error_flag) {  // エラー時
    if (buzzer_cnt > 20) {
      buzzer_cnt = 0;
      if (buzzer_state == false) {
        buzzer_state = true;
        actuator_buzzer_frq_on(2200);
      } else {
        buzzer_state = false;
        actuator_buzzer_off();
      }
    }
  } else if (canRxTimeoutDetection(can_raw)) {  // 内部通信切断時
    if (buzzer_cnt > 200) {
      buzzer_cnt = 0;
      if (buzzer_state == false) {
        buzzer_state = true;
        actuator_buzzer_frq_on(500 * buzzer_frq_offset__gain);
        buzzer_frq_offset__gain *= 1.122462;
        if (buzzer_frq_offset__gain > 1.2) {
          buzzer_frq_offset__gain = 1.0;
        }
      } else {
        buzzer_state = false;
        actuator_buzzer_off();
      }
    }
  } else if (isVisionLost(sys, con, ai_cmd)) {
    if (buzzer_cnt > 20) {
      buzzer_cnt = 0;
      if (buzzer_state == false) {
        buzzer_state = true;
        actuator_buzzer_frq_on(2000 * buzzer_frq_offset__gain);
        buzzer_frq_offset__gain *= 1.122462;
        if (buzzer_frq_offset__gain > 1.2) {
          buzzer_frq_offset__gain = 1.0;
        }
      } else {
        buzzer_state = false;
        actuator_buzzer_off();
      }
    }
  } else if (buzzer_state) {
    buzzer_state = false;
    actuator_buzzer_off();
  }
}