/*
 * G474 Mainブートローダーのentry pointとして更新要求、confirm、rollback、A/B起動を制御する。
 */
#include "board_io.h"
#include "boot_config.h"
#include "boot_image.h"
#include "boot_uart_update.h"
#include "stm32g474xx.h"

static void backup_access_enable(void)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN;
  (void)RCC->APB1ENR1;
  PWR->CR1 |= PWR_CR1_DBP;
  while ((PWR->CR1 & PWR_CR1_DBP) == 0U) {}
}

int main(void)
{
  board_io_init_safe();
  board_status_set_validating(true);
  backup_access_enable();
  const uint32_t request = TAMP->BKP0R;
  const uint32_t argument = TAMP->BKP1R;
  TAMP->BKP0R = 0U;
  TAMP->BKP1R = 0U;

  if (request == BOOT_REQUEST_UPDATE && argument <= BOOT_SLOT_B) boot_uart_update((boot_slot_t)argument);
  if (request == BOOT_REQUEST_CONFIRM && argument <= BOOT_SLOT_B) (void)boot_confirm_slot((boot_slot_t)argument);

  boot_slot_t slot;
  boot_image_metadata_t metadata;
  boot_control_t control;
  if (boot_select_slot(&slot, &metadata, &control)) {
    if (metadata.state == BOOT_IMAGE_STATE_PENDING) {
      const uint32_t attempts = control.preferred_slot == (uint32_t)slot ? control.boot_attempts + 1U : 1U;
      if (!boot_control_write(slot, metadata.generation, attempts)) goto invalid;
    }
    board_status_set_invalid(false);
    board_delay_cycles(800000U);
    board_status_set_validating(false);
    boot_jump_to_slot(slot);
  }

invalid:
  board_status_set_validating(false);
  board_status_set_invalid(true);
  for (;;) __NOP();
}
