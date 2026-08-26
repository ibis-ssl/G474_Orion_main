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

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t format_version;
  uint16_t record_size;
  uint32_t generation;
  uint32_t preferred_slot;
  uint32_t boot_attempts;
  uint32_t record_crc32c;
} boot_control_t;

_Static_assert(sizeof(boot_control_t) == 24U, "boot control layout changed");

uint32_t boot_slot_base(boot_slot_t slot);
uint32_t boot_slot_metadata_base(boot_slot_t slot);
bool boot_slot_is_valid(boot_slot_t slot, bool allow_pending, boot_image_metadata_t * metadata_out);
bool boot_control_read(boot_control_t * control);
bool boot_control_write(boot_slot_t preferred_slot, uint32_t generation, uint32_t boot_attempts);
bool boot_metadata_write(boot_slot_t slot, const boot_image_metadata_t * metadata);
bool boot_confirm_slot(boot_slot_t slot);
bool boot_select_slot(boot_slot_t * slot, boot_image_metadata_t * metadata, boot_control_t * control);
void boot_jump_to_slot(boot_slot_t slot) __attribute__((noreturn));
