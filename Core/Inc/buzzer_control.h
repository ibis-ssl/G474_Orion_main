#ifndef _BUZZER_CONTROL_H_
#define _BUZZER_CONTROL_H_

#include "management.h"

void buzzerControl(can_raw_t * can_raw, system_t * sys, connection_t * con, ai_cmd_t * ai_cmd);

#endif