/*
 * Slot A/Bのmetadata・vector・CRC32Cを検証し、control recordに従って安全な起動先を選択する。
 */
#include "boot_image.h"

#include "boot_crc32c.h"
#include "boot_flash.h"
#include "stm32g474xx.h"

#include <stddef.h>
#include <stdint.h>

uint32_t boot_slot_base(boot_slot_t slot) { return slot == BOOT_SLOT_A ? BOOT_SLOT_A_BASE : BOOT_SLOT_B_BASE; }
uint32_t boot_slot_metadata_base(boot_slot_t slot) { return slot == BOOT_SLOT_A ? BOOT_METADATA_SLOT_A_BASE : BOOT_METADATA_SLOT_B_BASE; }

static bool stack_pointer_is_valid(uint32_t value)
{
  return (((value >= BOOT_SRAM1_BASE && value <= BOOT_SRAM1_END) || (value >= BOOT_CCMRAM_BASE && value <= BOOT_CCMRAM_END)) && (value & 7U) == 0U);
}

bool boot_slot_is_valid(boot_slot_t slot, bool allow_pending, boot_image_metadata_t * metadata_out)
{
  const uint32_t base = boot_slot_base(slot);
  const boot_image_metadata_t metadata = *(const boot_image_metadata_t *)boot_slot_metadata_base(slot);
  if (metadata.magic != BOOT_IMAGE_METADATA_MAGIC || metadata.format_version != BOOT_IMAGE_METADATA_FORMAT || metadata.record_size != sizeof(metadata)) return false;
  if (metadata.slot != (uint32_t)slot || metadata.image_base != base || metadata.image_size < 8U || metadata.image_size > BOOT_SLOT_A_SIZE) return false;
  if (metadata.state != BOOT_IMAGE_STATE_CONFIRMED && (!allow_pending || metadata.state != BOOT_IMAGE_STATE_PENDING)) return false;
  if (boot_crc32c(&metadata, offsetof(boot_image_metadata_t, record_crc32c)) != metadata.record_crc32c) return false;
  const uint32_t stack = *(const uint32_t *)base;
  const uint32_t reset = *(const uint32_t *)(base + 4U);
  const uint32_t handler = reset & ~UINT32_C(1);
  if (!stack_pointer_is_valid(stack) || (reset & 1U) == 0U || handler < base || handler >= base + metadata.image_size) return false;
  if (boot_crc32c((const void *)base, metadata.image_size) != metadata.image_crc32c) return false;
  if (metadata_out != NULL) *metadata_out = metadata;
  return true;
}

bool boot_control_read(boot_control_t * control)
{
  const boot_control_t value = *(const boot_control_t *)BOOT_CONTROL_BASE;
  if (value.magic != BOOT_CONTROL_MAGIC || value.format_version != BOOT_CONTROL_FORMAT || value.record_size != sizeof(value) || value.preferred_slot > BOOT_SLOT_B) return false;
  if (boot_crc32c(&value, offsetof(boot_control_t, record_crc32c)) != value.record_crc32c) return false;
  if (control != NULL) *control = value;
  return true;
}

bool boot_control_write(boot_slot_t preferred_slot, uint32_t generation, uint32_t boot_attempts)
{
  boot_control_t value = {BOOT_CONTROL_MAGIC, BOOT_CONTROL_FORMAT, sizeof(value), generation, (uint32_t)preferred_slot, boot_attempts, 0U};
  value.record_crc32c = boot_crc32c(&value, offsetof(boot_control_t, record_crc32c));
  return boot_flash_erase_page(BOOT_CONTROL_BASE) && boot_flash_program(BOOT_CONTROL_BASE, (const uint8_t *)&value, sizeof(value));
}

bool boot_metadata_write(boot_slot_t slot, const boot_image_metadata_t * metadata)
{
  return metadata != NULL && boot_flash_erase_page(boot_slot_metadata_base(slot)) && boot_flash_program(boot_slot_metadata_base(slot), (const uint8_t *)metadata, sizeof(*metadata));
}

bool boot_confirm_slot(boot_slot_t slot)
{
  boot_image_metadata_t metadata;
  if (!boot_slot_is_valid(slot, true, &metadata) || metadata.state != BOOT_IMAGE_STATE_PENDING) return false;
  metadata.state = BOOT_IMAGE_STATE_CONFIRMED;
  metadata.record_crc32c = boot_crc32c(&metadata, offsetof(boot_image_metadata_t, record_crc32c));
  return boot_metadata_write(slot, &metadata) && boot_control_write(slot, metadata.generation, 0U);
}

bool boot_select_slot(boot_slot_t * slot, boot_image_metadata_t * metadata, boot_control_t * control_out)
{
  boot_image_metadata_t images[2];
  const bool valid[2] = {boot_slot_is_valid(BOOT_SLOT_A, true, &images[0]), boot_slot_is_valid(BOOT_SLOT_B, true, &images[1])};
  boot_control_t control = {0};
  const bool control_valid = boot_control_read(&control);
  uint32_t selected = control_valid ? control.preferred_slot : (valid[1] && (!valid[0] || images[1].generation > images[0].generation) ? 1U : 0U);
  if (!valid[selected]) selected ^= 1U;
  if (!valid[selected]) return false;
  if (images[selected].state == BOOT_IMAGE_STATE_PENDING && control_valid && control.preferred_slot == selected && control.boot_attempts >= 3U) {
    const uint32_t fallback = selected ^ 1U;
    if (!valid[fallback] || images[fallback].state != BOOT_IMAGE_STATE_CONFIRMED) return false;
    selected = fallback;
    if (!boot_control_write((boot_slot_t)selected, images[selected].generation, 0U)) return false;
    (void)boot_control_read(&control);
  }
  if (slot != NULL) *slot = (boot_slot_t)selected;
  if (metadata != NULL) *metadata = images[selected];
  if (control_out != NULL) *control_out = control;
  return true;
}

static void boot_branch(uint32_t stack, uint32_t reset) __attribute__((naked, noreturn));
static void boot_branch(uint32_t stack __attribute__((unused)), uint32_t reset __attribute__((unused)))
{
  __asm volatile("movs r2,#0\nmsr control,r2\nmsr basepri,r2\nmsr faultmask,r2\nisb\nmsr msp,r0\nmsr primask,r2\nbx r1\n");
}

void boot_jump_to_slot(boot_slot_t slot)
{
  const uint32_t base = boot_slot_base(slot);
  const uint32_t stack = *(const uint32_t *)base;
  const uint32_t reset = *(const uint32_t *)(base + 4U);
  __disable_irq();
  SysTick->CTRL = 0U; SysTick->LOAD = 0U; SysTick->VAL = 0U;
  for (uint32_t index = 0U; index < 8U; index++) { NVIC->ICER[index] = UINT32_MAX; NVIC->ICPR[index] = UINT32_MAX; }
  SCB->VTOR = base;
  __DSB(); __ISB();
  boot_branch(stack, reset);
}
