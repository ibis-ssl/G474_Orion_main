/* CM4から受けた更新chunkをCANノードへ配信する更新専用ゲートウェイの公開API。 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

void fw_update_gateway_start(uint8_t session);
void fw_update_gateway_uart_rx_byte(uint8_t value);
bool fw_update_gateway_can_rx(FDCAN_HandleTypeDef * hfdcan, uint32_t identifier, const uint8_t data[8]);
void fw_update_gateway_process(void);

