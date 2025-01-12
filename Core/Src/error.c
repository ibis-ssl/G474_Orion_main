#include "error.h"

#include <stdio.h>

/* USER CODE BEGIN Private defines */
enum {
  POWER_NONE = 0,
  POWER_UNDER_VOLTAGE = 0x0001,
  POWER_OVER_VOLTAGE = 0x0002,
  POWER_OVER_CURRENT = 0x0004,
  POWER_SHORT_CURCUIT = 0x0008,
  POWER_CHARGE_TIME = 0x0010,
  POWER_CHARGE_POWER = 0x0020,
  POWER_DISCHARGE = 0x0040,
  POWER_PARAMETER = 0x0080,
  POWER_COMMAND = 0x0100,
  POWER_NO_CAP = 0x0200,
  POWER_DISCHARGE_FAIL = 0x0400,
  POWER_GD_POWER_FAIL = 0x0800,
  POWER_COIL_OVER_HEAT = 0x1000,
  POWER_FET_OVER_HEAT = 0x2000,
};

enum {
  BLDC_NONE = 0,
  BLDC_UNDER_VOLTAGE = 0x0001,
  BLDC_OVER_CURRENT = 0x0002,
  BLDC_MOTOR_OVER_HEAT = 0x0004,
  BLDC_OVER_LOAD = 0x0008,
  BLDC_ENC_ERROR = 0x0010,
  BLDC_OVER_VOLTAGE = 0x0020,
  BLDC_FET_OVER_HEAT = 0x0040,
};

void convertErrorDataToStr(uint8_t id, uint16_t info, char * str_buf)
{
  int offset = 0;
  if (id == 100) {
    offset = sprintf(str_buf, "POWER : ");
    switch (info) {
      case POWER_NONE:
        sprintf(str_buf + offset, "no error");
        break;
      case POWER_UNDER_VOLTAGE:
        sprintf(str_buf + offset, "UNDER_VOLTAGE");
        break;
      case POWER_OVER_VOLTAGE:
        sprintf(str_buf + offset, "OVER_VOLTAGE");
        break;
      case POWER_OVER_CURRENT:
        sprintf(str_buf + offset, "OVER_CURRENT");
        break;
      case POWER_SHORT_CURCUIT:
        sprintf(str_buf + offset, "SHORT_CURCUIT");
        break;
      case POWER_CHARGE_TIME:
        sprintf(str_buf + offset, "CHARGE_TIME");
        break;
      case POWER_CHARGE_POWER:
        sprintf(str_buf + offset, "CHARGE_POWER");
        break;
      case POWER_DISCHARGE:
        sprintf(str_buf + offset, "DISCHARGE");
        break;
      case POWER_PARAMETER:
        sprintf(str_buf + offset, "PARAMETER");
        break;
      case POWER_COMMAND:
        sprintf(str_buf + offset, "COMMAND");
        break;
      case POWER_NO_CAP:
        sprintf(str_buf + offset, "NO_CAP");
        break;
      case POWER_DISCHARGE_FAIL:
        sprintf(str_buf + offset, "DISCHARGE_FAIL");
        break;
      case POWER_GD_POWER_FAIL:
        sprintf(str_buf + offset, "GD_POWER_FAIL");
        break;
      case POWER_COIL_OVER_HEAT:
        sprintf(str_buf + offset, "COIL_OVER_HEAT");
        break;
      case POWER_FET_OVER_HEAT:
        sprintf(str_buf + offset, "FET_OVER_HEAT");
        break;
      default:
        sprintf(str_buf + offset, "unknown info %d ", info);
        break;
    }
  } else {
    if (id == 0) {
      offset = sprintf(str_buf, "BLDC-RF : ");
    } else if (id == 1) {
      offset = sprintf(str_buf, "BLDC-RB : ");
    } else if (id == 2) {
      offset = sprintf(str_buf, "BLDC-LB : ");
    } else if (id == 3) {
      offset = sprintf(str_buf, "BLDC-LF : ");
    } else {
      sprintf(str_buf, "BLDC unknown id : %d info : %d ", id, info);
      return;
    }
    switch (info) {
      case BLDC_NONE:
        sprintf(str_buf + offset, "no error");
        break;
      case BLDC_UNDER_VOLTAGE:
        sprintf(str_buf + offset, "UNDER_VOLTAGE");
        break;
      case BLDC_OVER_CURRENT:
        sprintf(str_buf + offset, "OVER_CURRENT");
        break;
      case BLDC_MOTOR_OVER_HEAT:
        sprintf(str_buf + offset, "MOTOR_OVER_HEAT");
        break;
      case BLDC_OVER_LOAD:
        sprintf(str_buf + offset, "OVER_LOAD");
        break;
      case BLDC_ENC_ERROR:
        sprintf(str_buf + offset, "ENC_ERROR");
        break;
      case BLDC_OVER_VOLTAGE:
        sprintf(str_buf + offset, "OVER_VOLTAGE");
        break;
      case BLDC_FET_OVER_HEAT:
        sprintf(str_buf + offset, "FET_OVER_HEAT");
        break;
      default:
        sprintf(str_buf + offset, "unknown info %d ", info);
        break;
    }
  }
}