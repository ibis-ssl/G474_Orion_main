#include "stop_state_control.h"

#include "actuator.h"

bool isOverCurrentError(uint16_t error_id, uint16_t error_info) { return error_id < 4 && error_info == BLDC_ERROR_OVER_CURRENT; }

void stopStateControl(system_t * sys, uint8_t main_mode_sw)
{
  if (sys->error_flag) {
    // 一定回数はリセットを許容する
    if (isOverCurrentError(sys->error_id, sys->error_info) && sys->error_resume_cnt < 10) {
      sys->error_flag = false;
      sys->error_info = 0;
      sys->error_value = 0;

      sys->error_resume_cnt++;

      // しばらくstopに落とす
      requestStop(sys, 3.0);

      // OFFコマンドでリセット
      actuatorPower_ONOFF(0);
    } else {
      sys->main_mode = MAIN_MODE_ERROR;
      //resetLocalSpeedControl(&ai_cmd);
    }
  } else {
    sys->main_mode = main_mode_sw;
  }

  if (isStopRequested(sys) || sys->can_timeout || !sys->enc_initialized) {
    //resetLocalSpeedControl(&ai_cmd);
    sys->stop_flag = true;
  } else {
    sys->stop_flag = false;
  }
}

void requestStop(system_t * sys, float time_sec)
{
  sys->stop_flag_request_time = sys->system_time_ms + (uint32_t)(time_sec * MAIN_LOOP_CYCLE);  // 前回のタイムアウト時から1.0s間は動かさない
}

bool isStopRequested(system_t * sys) { return sys->system_time_ms < sys->stop_flag_request_time; }
