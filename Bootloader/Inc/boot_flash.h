/*
 * G474ブートローダーがapplication slotとmetadata/control pageを安全に消去・書込みする。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

bool boot_flash_erase_page(uint32_t address);
bool boot_flash_program(uint32_t address, const uint8_t * data, uint32_t length);
