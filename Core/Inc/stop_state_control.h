#ifndef STOP_STATE_CONTROL_H_
#define STOP_STATE_CONTROL_H_

#include "management.h"

void stopStateControl(system_t * sys, uint8_t main_mode_sw);
void requestStop(system_t * sys, float time_sec);
bool isStopRequested(system_t * sys);
#endif  //STOP_STATE_CONTROL_H_