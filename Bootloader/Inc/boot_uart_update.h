/*
 * CM4からUSART2経由でinactive Main slotを更新するOFW1 protocolを提供する。
 */
#pragma once

#include "boot_config.h"

void boot_uart_update(boot_slot_t running_slot) __attribute__((noreturn));
