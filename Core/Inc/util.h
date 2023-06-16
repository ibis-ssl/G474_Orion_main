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
float dtor(float x);
float rtod(float x);
__attribute__ ((weak)) int sign(int val);
int fsign(float val);
float constrain(float x,float a,float b);
float area(float value,float shita,float ue);
float max(float a,float b);
float min(float a,float b);
float abs_max(float a,float b);
float abs_min(float a,float b);
float floatlimit(float mae,float val,float ato);

int uchar4_to_int(unsigned char *value);
void int_to_uchar4(unsigned char *value,int int_value);

unsigned short uchar2_to_ushort(unsigned char *value);
void ushort_to_uchar2(unsigned char *value,unsigned short short_value);

float uchar4_to_float(unsigned char *value);
void float_to_uchar4(unsigned char *value,float float_value);

//added
float deg_to_radian(float deg);
float radian_to_deg(float radian);

#ifdef __cplusplus
}
#endif
