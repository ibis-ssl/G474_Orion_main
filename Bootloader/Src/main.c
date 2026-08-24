/*
 * G474 MainアプリケーションブートローダーM1のentry pointを実装する。
 * 安全IOを維持し、confirmed Slot Aの検証成功時だけ通常アプリへ遷移する。
 */
#include "board_io.h"
#include "boot_image.h"
#include "stm32g474xx.h"

int main(void)
{
  board_io_init_safe();
  board_status_set_validating(true);

  if (boot_slot_a_is_valid()) {
    board_status_set_invalid(false);
    board_delay_cycles(800000U);
    board_status_set_validating(false);
    boot_jump_to_slot_a();
  }

  board_status_set_validating(false);
  board_status_set_invalid(true);

  for (;;) {
    __NOP();
  }
}

