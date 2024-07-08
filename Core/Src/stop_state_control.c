#include "stop_state_control.h"

#include "actuator.h"

void stopStateControl(system_t * sys, uint8_t main_mode, bool can_timeout, bool enc_initialized)
{
  if (sys->error_flag) {
    // 一定回数はリセットを許容する
    if (sys->error_id < 4 && sys->error_info == BLDC_ERROR_OVER_CURRENT && sys->error_resume_cnt < 10) {
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
    sys->main_mode = main_mode;
  }

  if (isStopRequested(sys) || can_timeout || !enc_initialized) {
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
