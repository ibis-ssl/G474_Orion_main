#ifndef ERROR_CONVERT_H_
#define ERROR_CONVERT_H_
#include <stdint.h>

void convertErrorDataToStr(uint8_t id, uint16_t info, uint8_t str_buf[]);

#endif