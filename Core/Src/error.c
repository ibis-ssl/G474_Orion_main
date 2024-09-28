#include "error.h"

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
};

void convertErrorDataToStr(uint8_t id, uint16_t info, uint8_t str_buf[])
{

  if (id == 100) {
    switch (info) {
      case POWER_NONE:
        break;
      case POWER_UNDER_VOLTAGE:
        break;
      case POWER_OVER_VOLTAGE:
        break;
      case POWER_OVER_CURRENT:
        break;
      case POWER_SHORT_CURCUIT:
        break;
      case POWER_CHARGE_TIME:
        break;
      case POWER_CHARGE_POWER:
        break;
      case POWER_DISCHARGE:
        break;
      case POWER_PARAMETER:
        break;
      case POWER_COMMAND:
        break;
      case POWER_NO_CAP:
        break;
      case POWER_DISCHARGE_FAIL:
        break;
      case POWER_GD_POWER_FAIL:
        break;
      case POWER_COIL_OVER_HEAT:
        break;
      case POWER_FET_OVER_HEAT:
        break;
      default:
        break;
    }
  } else {
    switch (info) {
      case BLDC_NONE:
        break;
      case BLDC_UNDER_VOLTAGE:
        break;
      case BLDC_OVER_CURRENT:
        break;
      case BLDC_MOTOR_OVER_HEAT:
        break;
      case BLDC_OVER_LOAD:
        break;
      case BLDC_ENC_ERROR:
        break;
      case BLDC_OVER_VOLTAGE:
        break;
      default:
        break;
    }
  }
}