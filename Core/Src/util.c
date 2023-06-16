#include "util.h"
#include "math.h"
#include <stdio.h>

float dtor(float x){
	return (2*M_PI*x/360);	//x/360*2*PI
}

float rtod(float x){
	return (x*360/(2*M_PI));	//x/(2*PI)*360
}

int sign(int val)
{
	if(val>=0) return 1;
	else return -1;
}

int fsign(float val)
{
	if(val>=0.0) return 1;
	else return -1;
}

float floatlimit(float max,float val,float min)
{
	if(min>=val) return min;
	if(max<=val) return max;
	return val;
}

float constrain(float x,float a,float b){
	if(x<a) return a;
	else if(b<x) return b;
	else return x;
}

float area(float value,float shita,float ue)
{
	int i=0,limit=100;
	if(shita>=ue) return 0;
	for(i=0;value<shita&&i<limit;i++) {value+=(ue-shita);}
	for(i=0;ue<value&&i<limit;i++) {value-=(ue-shita);}
	return value;
}

float max(float a,float b){
	if(a>b) return a;
	else return b;
}

float min(float a,float b){
	if(a<b) return a;
	else return b;
}

float abs_max(float a,float b){
    if(fabs(a)>fabs(b)) return a;
    else return b;
}

float abs_min(float a,float b){
    if(fabs(a)<fabs(b)) return a;
    else return b;
}

typedef union{
    int int_value;
    unsigned char char4_value[4];
}Int_char4;

int uchar4_to_int(unsigned char *value){
    Int_char4 tmp;
    tmp.char4_value[0]=value[0];
    tmp.char4_value[1]=value[1];
    tmp.char4_value[2]=value[2];
    tmp.char4_value[3]=value[3];
    return tmp.int_value;
}

void int_to_uchar4(unsigned char *value,int int_value){
    Int_char4 tmp;
    tmp.int_value=int_value;
    value[0]=tmp.char4_value[0];
    value[1]=tmp.char4_value[1];
    value[2]=tmp.char4_value[2];
    value[3]=tmp.char4_value[3];
}

typedef union{
    unsigned short short_value;
    unsigned char char2_value[2];
}UShort_char2;


unsigned short uchar2_to_ushort(unsigned char *value){
    UShort_char2 tmp;
    tmp.char2_value[0]=value[0];
    tmp.char2_value[1]=value[1];
    return tmp.short_value;
}

void ushort_to_uchar2(unsigned char *value,unsigned short short_value){
    UShort_char2 tmp;
    tmp.short_value=short_value;
    value[0]=tmp.char2_value[0];
    value[1]=tmp.char2_value[1];
}

typedef union{
    float float_value;
    unsigned char char4_value[4];
}Float_char4;

float uchar4_to_float(unsigned char *value){
    Float_char4 tmp;
    tmp.char4_value[0]=value[0];
    tmp.char4_value[1]=value[1];
    tmp.char4_value[2]=value[2];
    tmp.char4_value[3]=value[3];
    return tmp.float_value;
}

void float_to_uchar4(unsigned char *value,float float_value){
    Float_char4 tmp;
    tmp.float_value=float_value;
    value[0]=tmp.char4_value[0];
    value[1]=tmp.char4_value[1];
    value[2]=tmp.char4_value[2];
    value[3]=tmp.char4_value[3];
}

float deg_to_radian(float deg){
	return (deg /180)*M_PI;
}


float radian_to_deg(float radian){
	return (radian * 180) / M_PI;
}
