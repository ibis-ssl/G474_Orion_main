#include "util.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

float dtor(float x)
{
  return (2 * M_PI * x / 360);  //x/360*2*PI
}

float rtod(float x)
{
  return (x * 360 / (2 * M_PI));  //x/(2*PI)*360
}

int sign(int val)
{
  if (val >= 0)
    return 1;
  else
    return -1;
}

int fsign(float val)
{
  if (val >= 0.0)
    return 1;
  else
    return -1;
}

float floatlimit(float max, float val, float min)
{
  if (min >= val) return min;
  if (max <= val) return max;
  return val;
}

float constrain(float x, float a, float b)
{
  if (x < a)
    return a;
  else if (b < x)
    return b;
  else
    return x;
}

float area(float value, float shita, float ue)
{
  int i = 0, limit = 100;
  if (shita >= ue) return 0;
  for (i = 0; value < shita && i < limit; i++) {
    value += (ue - shita);
  }
  for (i = 0; ue < value && i < limit; i++) {
    value -= (ue - shita);
  }
  return value;
}

float max(float a, float b)
{
  if (a > b)
    return a;
  else
    return b;
}

float min(float a, float b)
{
  if (a < b)
    return a;
  else
    return b;
}

float abs_max(float a, float b)
{
  if (fabsf(a) > fabsf(b))
    return a;
  else
    return b;
}

float abs_min(float a, float b)
{
  if (fabsf(a) < fabsf(b))
    return a;
  else
    return b;
}

typedef union {
  int int_value;
  unsigned char char4_value[4];
} Int_char4;

int uchar4_to_int(unsigned char * value)
{
  Int_char4 tmp;
  tmp.char4_value[0] = value[0];
  tmp.char4_value[1] = value[1];
  tmp.char4_value[2] = value[2];
  tmp.char4_value[3] = value[3];
  return tmp.int_value;
}

void int_to_uchar4(unsigned char * value, int int_value)
{
  Int_char4 tmp;
  tmp.int_value = int_value;
  value[0] = tmp.char4_value[0];
  value[1] = tmp.char4_value[1];
  value[2] = tmp.char4_value[2];
  value[3] = tmp.char4_value[3];
}

typedef union {
  unsigned short short_value;
  unsigned char char2_value[2];
} UShort_char2;

unsigned short uchar2_to_ushort(unsigned char * value)
{
  UShort_char2 tmp;
  tmp.char2_value[0] = value[0];
  tmp.char2_value[1] = value[1];
  return tmp.short_value;
}

void ushort_to_uchar2(unsigned char * value, unsigned short short_value)
{
  UShort_char2 tmp;
  tmp.short_value = short_value;
  value[0] = tmp.char2_value[0];
  value[1] = tmp.char2_value[1];
}

typedef union {
  float float_value;
  unsigned char char4_value[4];
} Float_char4;

float uchar4_to_float(unsigned char * value)
{
  Float_char4 tmp;
  tmp.char4_value[0] = value[0];
  tmp.char4_value[1] = value[1];
  tmp.char4_value[2] = value[2];
  tmp.char4_value[3] = value[3];
  return tmp.float_value;
}

void float_to_uchar4(unsigned char * value, float float_value)
{
  Float_char4 tmp;
  tmp.float_value = float_value;
  value[0] = tmp.char4_value[0];
  value[1] = tmp.char4_value[1];
  value[2] = tmp.char4_value[2];
  value[3] = tmp.char4_value[3];
}

float deg_to_radian(float deg)
{
  return (deg / 180) * M_PI;
}

float radian_to_deg(float radian)
{
  return (radian * 180) / M_PI;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float normalizeAngle(float angle_rad)
{
  while (angle_rad > M_PI) {
    angle_rad -= 2.0f * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0f * M_PI;
  }
  return angle_rad;
}

float getAngleDiff(float angle_rad1, float angle_rad2)
{
  angle_rad1 = normalizeAngle(angle_rad1);
  angle_rad2 = normalizeAngle(angle_rad2);
  if (fabsf(angle_rad1 - angle_rad2) > M_PI) {
    if (angle_rad1 > angle_rad2) {
      return angle_rad1 - (angle_rad2 + 2 * M_PI);
    } else {
      return (angle_rad1 + 2 * M_PI) - angle_rad2;
    }
  } else {
    return angle_rad1 - angle_rad2;
  }
}

void convertGlobalToLocal(float global[], float local[], float yaw_rad)
{
  local[0] = global[0] * cosf(-yaw_rad) - global[1] * sinf(-yaw_rad);
  local[1] = global[0] * sinf(-yaw_rad) + global[1] * cosf(-yaw_rad);
}

void convertLocalToGlobal(float local[], float global[], float yaw_rad)
{
  global[0] = local[0] * cosf(yaw_rad) - local[1] * sinf(yaw_rad);
  global[1] = local[0] * sinf(yaw_rad) + local[1] * cosf(yaw_rad);
}

float calcScalar(float x, float y)
{
  return sqrtf(x * x + y * y);
}

void clampScalarSize(float xy_value[], float scalar_limit)
{
  float scalar = calcScalar(xy_value[0], xy_value[1]);
  if (scalar_limit < scalar) {
    xy_value[0] *= scalar_limit / scalar;
    xy_value[1] *= scalar_limit / scalar;
  }
}

float clampSize(float in, float max)
{
  if (in > max) {
    in = max;
  }
  if (in < -max) {
    in = -max;
  }
  return in;
}

bool swForwardPushed(uint16_t sw_raw_data)
{
  return sw_raw_data <= 3000 && sw_raw_data > 2000;
}
bool swCentorPushed(uint16_t sw_raw_data)
{
  return sw_raw_data <= 100;
}
bool swBackPushed(uint16_t sw_raw_data)
{
  return sw_raw_data <= 500 && sw_raw_data > 100;
}
bool swRightPushed(uint16_t sw_raw_data)
{
  return sw_raw_data <= 2000 && sw_raw_data > 500;
}
bool swLeftPushed(uint16_t sw_raw_data)
{
  return sw_raw_data <= 3900 && sw_raw_data > 3000;
}

float two_to_float(uint8_t data[2])
{
  return (float)((data[0] << 8 | data[1]) - 32767.0) / 32767.0;
}
float two_to_int(uint8_t data[2])
{
  return ((data[0] << 8 | data[1]) - 32767.0);
}
