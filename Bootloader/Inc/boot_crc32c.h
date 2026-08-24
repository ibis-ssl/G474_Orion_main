/*
 * ブートmetadataとアプリイメージの転送破損を検出するCRC32Cを提供する。
 * reflected Castagnoli polynomial、初期値/最終XORとも0xFFFFFFFFを使用する。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

uint32_t boot_crc32c(const void * data, size_t length);

