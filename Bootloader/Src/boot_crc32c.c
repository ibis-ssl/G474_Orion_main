/*
 * 小容量ブートローダー向けにtableを持たないCRC32C計算を実装する。
 * build時のPython生成スクリプトと同じCastagnoli条件を使用する。
 */
#include "boot_crc32c.h"

#include <stdint.h>

#define CRC32C_REFLECTED_POLYNOMIAL UINT32_C(0x82F63B78)

uint32_t boot_crc32c(const void * data, size_t length)
{
  const uint8_t * bytes = (const uint8_t *)data;
  uint32_t crc = UINT32_C(0xFFFFFFFF);

  for (size_t index = 0; index < length; index++) {
    crc ^= bytes[index];
    for (uint32_t bit = 0; bit < 8U; bit++) {
      const uint32_t lsb_mask = (uint32_t)(-(int32_t)(crc & 1U));
      crc = (crc >> 1U) ^ (CRC32C_REFLECTED_POLYNOMIAL & lsb_mask);
    }
  }

  return crc ^ UINT32_C(0xFFFFFFFF);
}

