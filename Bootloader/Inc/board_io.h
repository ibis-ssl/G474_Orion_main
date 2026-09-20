/*
 * G474 Main基板の全GPIOをブートローダー用の明示的な安全状態へ設定する。
 * HAL初期化前に呼び出せるようCMSISレジスタだけを使用する。
 */
#pragma once

#include <stdbool.h>

void board_io_init_safe(void);
void board_status_set_validating(bool enabled);
void board_status_set_invalid(bool enabled);
void board_delay_cycles(unsigned int cycles);

