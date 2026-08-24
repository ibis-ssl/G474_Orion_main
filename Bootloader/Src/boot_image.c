/*
 * 固定metadataを使ってSlot Aの範囲、vector、CRC32Cを検証し、通常アプリへjumpする。
 * 不正なaddressへの分岐を防ぎ、検証失敗時は呼び出し元へfalseを返す。
 */
#include "boot_image.h"

#include "boot_crc32c.h"
#include "stm32g474xx.h"

#include <stddef.h>
#include <stdint.h>

static const boot_image_metadata_t * const slot_a_metadata = (const boot_image_metadata_t *)BOOT_METADATA_BASE;

static bool stack_pointer_is_valid(uint32_t stack_pointer)
{
  const bool in_sram1 = stack_pointer >= BOOT_SRAM1_BASE && stack_pointer <= BOOT_SRAM1_END;
  const bool in_ccmram = stack_pointer >= BOOT_CCMRAM_BASE && stack_pointer <= BOOT_CCMRAM_END;
  return (in_sram1 || in_ccmram) && (stack_pointer & 7U) == 0U;
}

static bool reset_handler_is_valid(uint32_t reset_handler, uint32_t image_end)
{
  const uint32_t handler_address = reset_handler & ~UINT32_C(1);
  return (reset_handler & 1U) != 0U && handler_address >= BOOT_SLOT_A_BASE && handler_address < image_end;
}

bool boot_slot_a_is_valid(void)
{
  const boot_image_metadata_t metadata = *slot_a_metadata;

  if (metadata.magic != BOOT_IMAGE_METADATA_MAGIC || metadata.format_version != BOOT_IMAGE_METADATA_FORMAT || metadata.record_size != sizeof(boot_image_metadata_t)) {
    return false;
  }
  if (metadata.state != BOOT_IMAGE_STATE_CONFIRMED || metadata.slot != BOOT_SLOT_A || metadata.image_base != BOOT_SLOT_A_BASE) {
    return false;
  }
  if (metadata.image_size < 8U || metadata.image_size > BOOT_SLOT_A_SIZE) {
    return false;
  }

  const uint32_t record_crc = boot_crc32c(&metadata, offsetof(boot_image_metadata_t, record_crc32c));
  if (record_crc != metadata.record_crc32c) {
    return false;
  }

  const uint32_t image_end = metadata.image_base + metadata.image_size;
  if (image_end < metadata.image_base || image_end > BOOT_SLOT_A_BASE + BOOT_SLOT_A_SIZE) {
    return false;
  }

  const uint32_t initial_stack_pointer = *(const uint32_t *)BOOT_SLOT_A_BASE;
  const uint32_t reset_handler = *(const uint32_t *)(BOOT_SLOT_A_BASE + 4U);
  if (!stack_pointer_is_valid(initial_stack_pointer) || !reset_handler_is_valid(reset_handler, image_end)) {
    return false;
  }

  return boot_crc32c((const void *)BOOT_SLOT_A_BASE, metadata.image_size) == metadata.image_crc32c;
}

static void boot_branch(uint32_t stack_pointer, uint32_t reset_handler) __attribute__((naked, noreturn));
static void boot_branch(uint32_t stack_pointer __attribute__((unused)), uint32_t reset_handler __attribute__((unused)))
{
  /* Reset_Handlerへ渡す前に、bootloaderが変更した例外maskと実行権限をreset相当へ戻す。 */
  __asm volatile(
    "movs r2, #0\n"
    "msr control, r2\n"
    "msr basepri, r2\n"
    "msr faultmask, r2\n"
    "isb\n"
    "msr msp, r0\n"
    "msr primask, r2\n"
    "bx r1\n");
}

void boot_jump_to_slot_a(void)
{
  const uint32_t initial_stack_pointer = *(const uint32_t *)BOOT_SLOT_A_BASE;
  const uint32_t reset_handler = *(const uint32_t *)(BOOT_SLOT_A_BASE + 4U);

  __disable_irq();
  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;

  for (uint32_t index = 0; index < 8U; index++) {
    NVIC->ICER[index] = UINT32_MAX;
    NVIC->ICPR[index] = UINT32_MAX;
  }

  SCB->VTOR = BOOT_SLOT_A_BASE;
  __DSB();
  __ISB();
  boot_branch(initial_stack_pointer, reset_handler);
}
