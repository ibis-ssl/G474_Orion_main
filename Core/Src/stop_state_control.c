#include "stop_state_control.h"

#include "actuator.h"

bool isOverCurrentError(uint16_t error_id, uint16_t error_info) { return error_id < 4 && error_info == BLDC_ERROR_OVER_CURRENT; }

void stopStateControl(system_t * sys, uint8_t main_mode_sw)
{
  if (sys->error_flag) {
    // 一定回数はリセットを許容する
    if (isOverCurrentError(sys->current_error.id, sys->current_error.info) && sys->current_error.resume_cnt < 10) {
      sys->error_flag = false;
      sys->current_error.id = 0;
      sys->current_error.info = 0;

      sys->current_error.resume_cnt++;

      // しばらくstopに落とす
      requestStop(sys, 3000);
    }
    sys->main_mode = MAIN_MODE_ERROR;
  } else {
    sys->main_mode = main_mode_sw;
  }

  if (!sys->enc_initialized) {
    requestStop(sys, 100);
    // エンコーダ初期値を受け取った後、stopを挟むため
  }

  if (isStopRequested(sys) || sys->can_timeout) {
    sys->stop_flag = true;
  } else {
    sys->stop_flag = false;
  }
}

void requestStop(system_t * sys, uint32_t time_ms)
{
  //より短い値で上書きしないためのブロック
  if (sys->stop_flag_request_time > sys->system_time_ms + time_ms) {
    return;
  }
  sys->stop_flag_request_time = sys->system_time_ms + (uint32_t)(time_ms);
}

bool isStopRequested(system_t * sys) { return sys->system_time_ms < sys->stop_flag_request_time; }
