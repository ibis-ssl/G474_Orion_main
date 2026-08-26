/*
 * STM32G474 dual-bank Flashのpage消去と64-bit programmingをbare-metalで実装する。
 */
#include "boot_flash.h"

#include "boot_config.h"
#include "stm32g474xx.h"

#include <stddef.h>
#include <stdint.h>

#define FLASH_KEY1_VALUE UINT32_C(0x45670123)
#define FLASH_KEY2_VALUE UINT32_C(0xCDEF89AB)
#define FLASH_ERROR_MASK (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR)

static bool flash_wait(void)
{
  while ((FLASH->SR & FLASH_SR_BSY) != 0U) {}
  const uint32_t errors = FLASH->SR & FLASH_ERROR_MASK;
  FLASH->SR = FLASH_SR_EOP | FLASH_ERROR_MASK;
  return errors == 0U;
}

static bool flash_unlock(void)
{
  if ((FLASH->CR & FLASH_CR_LOCK) != 0U) { FLASH->KEYR = FLASH_KEY1_VALUE; FLASH->KEYR = FLASH_KEY2_VALUE; }
  return (FLASH->CR & FLASH_CR_LOCK) == 0U;
}

static void flash_lock(void) { FLASH->CR |= FLASH_CR_LOCK; }

bool boot_flash_erase_page(uint32_t address)
{
  if (address < BOOT_SLOT_A_BASE || address >= BOOT_METADATA_BASE + BOOT_METADATA_SIZE || (address & (BOOT_FLASH_PAGE_SIZE - 1U)) != 0U) return false;
  if (!flash_wait() || !flash_unlock()) return false;
  const uint32_t absolute_page = (address - BOOTLOADER_FLASH_BASE) / BOOT_FLASH_PAGE_SIZE;
  const uint32_t page = absolute_page & UINT32_C(0x7F);
  FLASH->CR = FLASH_CR_PER | (page << FLASH_CR_PNB_Pos) | (absolute_page >= 128U ? FLASH_CR_BKER : 0U);
  FLASH->CR |= FLASH_CR_STRT;
  const bool ok = flash_wait();
  FLASH->CR = 0U;
  flash_lock();
  return ok;
}

bool boot_flash_program(uint32_t address, const uint8_t * data, uint32_t length)
{
  if ((address & 7U) != 0U || data == NULL || length == 0U) return false;
  if (!flash_wait() || !flash_unlock()) return false;
  for (uint32_t offset = 0U; offset < length; offset += 8U) {
    uint32_t low = UINT32_MAX, high = UINT32_MAX;
    const uint32_t remaining = length - offset;
    for (uint32_t index = 0U; index < 4U && index < remaining; index++) low = (low & ~(UINT32_C(0xFF) << (index * 8U))) | ((uint32_t)data[offset + index] << (index * 8U));
    for (uint32_t index = 4U; index < 8U && index < remaining; index++) high = (high & ~(UINT32_C(0xFF) << ((index - 4U) * 8U))) | ((uint32_t)data[offset + index] << ((index - 4U) * 8U));
    FLASH->CR = FLASH_CR_PG;
    *(volatile uint32_t *)(address + offset) = low;
    *(volatile uint32_t *)(address + offset + 4U) = high;
    if (!flash_wait() || *(const uint32_t *)(address + offset) != low || *(const uint32_t *)(address + offset + 4U) != high) { FLASH->CR = 0U; flash_lock(); return false; }
  }
  FLASH->CR = 0U;
  flash_lock();
  return true;
}
