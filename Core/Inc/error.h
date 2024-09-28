#ifndef ERROR_CONVERT_H_
#define ERROR_CONVERT_H_
#include <stdint.h>

void convertErrorDataToStr(uint8_t id, uint16_t info, unsigned char * str_buf);

#endif