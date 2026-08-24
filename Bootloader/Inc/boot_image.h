/*
 * G474 Slot Aのmetadata、vector、CRCを検証し、安全に通常アプリへ遷移する。
 * M1ではconfirmed Slot Aだけを起動対象とし、A/B選択は後続段階で拡張する。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "boot_config.h"

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t format_version;
  uint16_t record_size;
  uint32_t generation;
  uint32_t state;
  uint32_t slot;
  uint32_t image_base;
  uint32_t image_size;
  uint32_t image_crc32c;
  uint32_t record_crc32c;
} boot_image_metadata_t;

_Static_assert(sizeof(boot_image_metadata_t) == 36U, "boot metadata layout changed");

bool boot_slot_a_is_valid(void);
void boot_jump_to_slot_a(void) __attribute__((noreturn));

