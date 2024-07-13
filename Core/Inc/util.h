#pragma once
/*******************************************
 * util ver1.0 2015/1/11
 * This is a program made for mathematically calculation.
 *
 * [Dependency]
 * none
 *
 * [Note]
 * Coordinate functions will not support well in the future.
 * To operate coordinate, using "utilplus" is recommended.
 *
 * [Author]
 * Tomoki Nagatani
 *
 * [Change history]
 * ver1.1 2015/ 3/ 4 Add converter for int,unsigned short,short
 * ver1.1 2015/ 2/ 7 Add weak to sign, for utilplus1.0.
 * ver1.0 2015/ 1/11 C++ include capable.
 ******************************************/

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#include "stdbool.h"

float dtor(float x);
float rtod(float x);
__attribute__((weak)) int sign(int val);
int fsign(float val);
float constrain(float x, float a, float b);
float area(float value, float shita, float ue);
float max(float a, float b);
float min(float a, float b);
float abs_max(float a, float b);
float abs_min(float a, float b);
float floatlimit(float mae, float val, float ato);

int uchar4_to_int(unsigned char * value);
void int_to_uchar4(unsigned char * value, int int_value);

unsigned short uchar2_to_ushort(unsigned char * value);
void ushort_to_uchar2(unsigned char * value, unsigned short short_value);

float uchar4_to_float(unsigned char * value);
void float_to_uchar4(unsigned char * value, float float_value);

//added
float deg_to_radian(float deg);
float radian_to_deg(float radian);

// add(2)
long map(long x, long in_min, long in_max, long out_min, long out_max);
float getAngleDiff(float angle_rad1, float angle_rad2);
float normalizeAngle(float angle_rad);
float two_to_float(uint8_t data[2]);
float two_to_int(uint8_t data[2]);

bool swCentorPushed(uint16_t sw_raw_data);
bool swRightPushed(uint16_t sw_raw_data);
bool swLeftPushed(uint16_t sw_raw_data);
bool swForwardPushed(uint16_t sw_raw_data);
bool swBackPushed(uint16_t sw_raw_data);

void convertGlobalToLocal(float global[], float local[], float yaw_rad);
void convertLocalToGlobal(float local[], float global[], float yaw_rad);

float calcScalar(float x, float y);
void clampScalarSize(float xy_value[], float scalar_limit);

#ifdef __cplusplus
}
#endif
