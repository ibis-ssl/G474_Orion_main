/* CM4の可変長UART更新プロトコルを解析し、Subへ896-byte単位でCAN配信する。 */
#include "fw_update_gateway.h"

#include <string.h>

#include "fdcan.h"
#include "usart.h"

#define UART_MAGIC_0 'O'
#define UART_MAGIC_1 'F'
#define UART_MAGIC_2 'W'
#define UART_MAGIC_3 '2'
#define UART_VERSION 2U
#define UART_HEADER_SIZE 12U
#define UART_TRAILER_SIZE 4U
#define UART_MAX_PAYLOAD 907U
#define UART_BUFFER_SIZE (UART_HEADER_SIZE + UART_MAX_PAYLOAD + UART_TRAILER_SIZE)

#define MSG_ENTER 1U
#define MSG_BEGIN 2U
#define MSG_CHUNK 3U
#define MSG_FINALIZE 4U
#define MSG_REBOOT 5U
#define MSG_STATUS 6U

#define GATEWAY_OK 0U
#define GATEWAY_FRAME_ERROR 1U
#define GATEWAY_CAN_TIMEOUT 2U
#define GATEWAY_NODE_ERROR 3U
#define GATEWAY_RANGE_ERROR 4U
#define GATEWAY_SEQUENCE_ERROR 5U

#define CAN_COMMAND_ID 0x610U
#define CAN_DATA_ID_BASE 0x480U
#define CAN_RESPONSE_ID 0x654U
#define SUB_NODE_ID 4U
#define CHUNK_CAPACITY 896U

extern volatile bool fw_gateway_active;

static uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
static volatile uint16_t uart_rx_index;
static volatile uint16_t uart_rx_expected;
static volatile bool uart_frame_ready;
static volatile bool uart_rx_overflow;
static volatile uint32_t uart_last_byte_tick;

static volatile uint8_t can_reply[8];
static volatile uint32_t can_reply_counter;
static uint8_t update_session;

static bool last_response_valid;
static uint16_t last_sequence;
static uint8_t last_response_type;
static uint8_t last_response_payload[8];
static uint16_t last_response_length;

static uint16_t load_u16(const uint8_t * value)
{
  return (uint16_t)value[0] | ((uint16_t)value[1] << 8U);
}

static uint32_t load_u32(const uint8_t * value)
{
  return (uint32_t)value[0] | ((uint32_t)value[1] << 8U) | ((uint32_t)value[2] << 16U) | ((uint32_t)value[3] << 24U);
}

static void store_u16(uint8_t * target, uint16_t value)
{
  target[0] = (uint8_t)value;
  target[1] = (uint8_t)(value >> 8U);
}

static void store_u32(uint8_t * target, uint32_t value)
{
  target[0] = (uint8_t)value;
  target[1] = (uint8_t)(value >> 8U);
  target[2] = (uint8_t)(value >> 16U);
  target[3] = (uint8_t)(value >> 24U);
}

static uint16_t crc16_ccitt(const uint8_t * data, uint32_t length)
{
  uint16_t crc = UINT16_C(0xFFFF);
  for (uint32_t i = 0U; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8U;
    for (uint32_t bit = 0U; bit < 8U; bit++) crc = (crc & UINT16_C(0x8000)) != 0U ? (uint16_t)((crc << 1U) ^ UINT16_C(0x1021)) : (uint16_t)(crc << 1U);
  }
  return crc;
}

static uint32_t crc32c(const uint8_t * data, uint32_t length)
{
  uint32_t crc = UINT32_C(0xFFFFFFFF);
  for (uint32_t i = 0U; i < length; i++) {
    crc ^= data[i];
    for (uint32_t bit = 0U; bit < 8U; bit++) crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? UINT32_C(0x82F63B78) : 0U);
  }
  return ~crc;
}

static void reset_uart_parser(void)
{
  uart_rx_index = 0U;
  uart_rx_expected = 0U;
  uart_rx_overflow = false;
}

void fw_update_gateway_start(uint8_t session)
{
  update_session = session;
  uart_frame_ready = false;
  last_response_valid = false;
  reset_uart_parser();
}

void fw_update_gateway_uart_rx_byte(uint8_t value)
{
  static const uint8_t magic[4] = {UART_MAGIC_0, UART_MAGIC_1, UART_MAGIC_2, UART_MAGIC_3};
  uart_last_byte_tick = HAL_GetTick();
  if (uart_frame_ready) {
    uart_rx_overflow = true;
    return;
  }

  if (uart_rx_index < 4U) {
    if (value == magic[uart_rx_index]) {
      uart_rx_buffer[uart_rx_index++] = value;
    } else {
      uart_rx_index = value == magic[0] ? 1U : 0U;
      if (uart_rx_index == 1U) uart_rx_buffer[0] = value;
    }
    return;
  }

  if (uart_rx_index >= UART_BUFFER_SIZE) {
    uart_rx_overflow = true;
    reset_uart_parser();
    return;
  }
  uart_rx_buffer[uart_rx_index++] = value;

  if (uart_rx_index == UART_HEADER_SIZE) {
    const uint16_t payload_length = load_u16(&uart_rx_buffer[8]);
    if (uart_rx_buffer[4] != UART_VERSION || payload_length > UART_MAX_PAYLOAD ||
        crc16_ccitt(uart_rx_buffer, 10U) != load_u16(&uart_rx_buffer[10])) {
      reset_uart_parser();
      return;
    }
    uart_rx_expected = (uint16_t)(UART_HEADER_SIZE + payload_length + UART_TRAILER_SIZE);
  }

  if (uart_rx_expected != 0U && uart_rx_index == uart_rx_expected) {
    __DMB();
    uart_frame_ready = true;
  }
}

bool fw_update_gateway_can_rx(FDCAN_HandleTypeDef * hfdcan, uint32_t identifier, const uint8_t data[8])
{
  if (!fw_gateway_active || hfdcan->Instance != FDCAN1 || identifier != CAN_RESPONSE_ID) return false;
  memcpy((void *)can_reply, data, 8U);
  __DMB();
  can_reply_counter++;
  return true;
}

static bool fdcan_send_wait(uint32_t identifier, const uint8_t data[8], uint32_t timeout_ms)
{
  const uint32_t start = HAL_GetTick();
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
    if (HAL_GetTick() - start >= timeout_ms) return false;
  }
  FDCAN_TxHeaderTypeDef header = {
    .Identifier = identifier,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0U,
  };
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) == HAL_OK;
}

static bool can_command(uint8_t command, uint8_t token, uint32_t value, uint32_t timeout_ms, uint8_t reply[8])
{
  const uint8_t request[8] = {command, SUB_NODE_ID, token, 0U, (uint8_t)value, (uint8_t)(value >> 8U), (uint8_t)(value >> 16U), (uint8_t)(value >> 24U)};
  for (uint32_t attempt = 0U; attempt < 5U; attempt++) {
    const uint32_t before = can_reply_counter;
    if (!fdcan_send_wait(CAN_COMMAND_ID, request, 100U)) continue;
    const uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout_ms) {
      if (can_reply_counter != before) {
        __DMB();
        memcpy(reply, (const void *)can_reply, 8U);
        if (reply[0] == (uint8_t)(command | 0x80U) && reply[2] == SUB_NODE_ID && reply[3] == token) return true;
      }
    }
  }
  return false;
}

static void send_uart_response(uint8_t type, uint16_t sequence, const uint8_t * payload, uint16_t payload_length)
{
  uint8_t frame[UART_HEADER_SIZE + 8U + UART_TRAILER_SIZE] = {UART_MAGIC_0, UART_MAGIC_1, UART_MAGIC_2, UART_MAGIC_3, UART_VERSION, type};
  store_u16(&frame[6], sequence);
  store_u16(&frame[8], payload_length);
  store_u16(&frame[10], crc16_ccitt(frame, 10U));
  if (payload_length > 0U) memcpy(&frame[UART_HEADER_SIZE], payload, payload_length);
  store_u32(&frame[UART_HEADER_SIZE + payload_length], crc32c(frame, UART_HEADER_SIZE + payload_length));
  /* CM4側が送信完了から受信待ちへ戻るためのturnaround時間を確保する。 */
  HAL_Delay(2U);
  for (uint32_t attempt = 0U; attempt < 3U; attempt++) {
    const uint32_t start = HAL_GetTick();
    while (huart2.gState != HAL_UART_STATE_READY && HAL_GetTick() - start < 100U) {}
    if (huart2.gState == HAL_UART_STATE_READY &&
        HAL_UART_Transmit(&huart2, frame, UART_HEADER_SIZE + payload_length + UART_TRAILER_SIZE, 100U) == HAL_OK) return;
  }
}

static void set_result(uint8_t payload[8], uint8_t gateway_status, const uint8_t node_reply[8])
{
  payload[0] = gateway_status;
  payload[1] = node_reply != NULL ? node_reply[1] : 0U;
  payload[2] = SUB_NODE_ID;
  payload[3] = node_reply != NULL ? node_reply[3] : update_session;
  if (node_reply != NULL) memcpy(&payload[4], &node_reply[4], 4U);
  else memset(&payload[4], 0, 4U);
}

static void handle_request(uint8_t type, const uint8_t * payload, uint16_t length, uint8_t result[8], bool * reset_after_reply)
{
  uint8_t reply[8] = {0};
  set_result(result, GATEWAY_OK, NULL);
  *reset_after_reply = false;

  if (type == MSG_ENTER) {
    if (length != 4U) { result[0] = GATEWAY_RANGE_ERROR; return; }
    update_session = payload[0];
    const uint8_t enter[8] = {'O', 'F', 'W', 'U', 'P', SUB_NODE_ID, 0U, update_session};
    for (uint32_t i = 0U; i < 10U; i++) {
      (void)fdcan_send_wait(0x600U, enter, 100U);
      HAL_Delay(25U);
    }
    if (!can_command(1U, update_session, 0U, 250U, reply)) { result[0] = GATEWAY_CAN_TIMEOUT; return; }
    set_result(result, reply[1] == 0U ? GATEWAY_OK : GATEWAY_NODE_ERROR, reply);
    return;
  }

  if (type == MSG_BEGIN) {
    if (length != 12U) { result[0] = GATEWAY_RANGE_ERROR; return; }
    update_session = payload[0];
    const uint32_t image_size = load_u32(&payload[4]);
    const uint32_t image_crc = load_u32(&payload[8]);
    if (!can_command(2U, update_session, image_size, 10000U, reply)) { result[0] = GATEWAY_CAN_TIMEOUT; return; }
    if (reply[1] != 0U) { set_result(result, GATEWAY_NODE_ERROR, reply); return; }
    if (!can_command(3U, update_session, image_crc, 1000U, reply)) { result[0] = GATEWAY_CAN_TIMEOUT; return; }
    set_result(result, reply[1] == 0U ? GATEWAY_OK : GATEWAY_NODE_ERROR, reply);
    return;
  }

  if (type == MSG_CHUNK) {
    if (length < 11U || length > 11U + CHUNK_CAPACITY) { result[0] = GATEWAY_RANGE_ERROR; return; }
    const uint32_t offset = load_u32(payload);
    const uint32_t chunk_crc = load_u32(&payload[4]);
    const uint16_t chunk_length = load_u16(&payload[8]);
    const uint8_t injection = payload[10];
    if (chunk_length == 0U || chunk_length > CHUNK_CAPACITY || length != 11U + chunk_length) { result[0] = GATEWAY_RANGE_ERROR; return; }
    const uint8_t token = (uint8_t)(update_session + (offset / CHUNK_CAPACITY));
    for (uint32_t chunk_attempt = 0U; chunk_attempt < 3U; chunk_attempt++) {
      if (!can_command(4U, token, offset, 500U, reply)) continue;
      if (reply[1] != 0U) {
        if (reply[1] == 2U && load_u32(&reply[4]) == offset + chunk_length) { set_result(result, GATEWAY_OK, reply); result[1] = 0U; return; }
        set_result(result, GATEWAY_NODE_ERROR, reply);
        return;
      }
      const uint32_t frame_count = ((uint32_t)chunk_length + 6U) / 7U;
      bool send_ok = true;
      for (uint32_t order = 0U; order < frame_count; order++) {
        const uint32_t sequence = (chunk_attempt == 0U && (injection & 4U) != 0U) ? frame_count - 1U - order : order;
        if (chunk_attempt == 0U && (injection & 1U) != 0U && sequence == 5U) continue;
        uint8_t data[8] = {token, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
        const uint32_t position = sequence * 7U;
        const uint32_t copy_length = chunk_length - position < 7U ? chunk_length - position : 7U;
        memcpy(&data[1], &payload[11U + position], copy_length);
        if (chunk_attempt == 0U && (injection & 8U) != 0U && sequence == 6U) data[1] ^= 1U;
        if (!fdcan_send_wait(CAN_DATA_ID_BASE + sequence, data, 100U)) { send_ok = false; break; }
        if (chunk_attempt == 0U && (injection & 2U) != 0U && sequence == 7U && !fdcan_send_wait(CAN_DATA_ID_BASE + sequence, data, 100U)) { send_ok = false; break; }
      }
      if (!send_ok) { result[0] = GATEWAY_CAN_TIMEOUT; return; }
      HAL_Delay(2U);
      if (!can_command(5U, token, chunk_crc, 500U, reply)) {
        if (can_command(1U, token, 0U, 250U, reply)) {
          const uint32_t confirmed = load_u32(&reply[4]);
          if (reply[1] == 0U && confirmed == offset + chunk_length) { set_result(result, GATEWAY_OK, reply); return; }
          if (reply[1] == 0U && confirmed == offset) continue;
        }
        result[0] = GATEWAY_CAN_TIMEOUT;
        return;
      }
      if (reply[1] == 0U) { set_result(result, GATEWAY_OK, reply); return; }
      if (reply[1] != 3U && reply[1] != 4U) { set_result(result, GATEWAY_NODE_ERROR, reply); return; }
    }
    set_result(result, GATEWAY_NODE_ERROR, reply);
    return;
  }

  if (type == MSG_FINALIZE) {
    if (!can_command(6U, update_session, 0U, 5000U, reply)) { result[0] = GATEWAY_CAN_TIMEOUT; return; }
    set_result(result, reply[1] == 0U ? GATEWAY_OK : GATEWAY_NODE_ERROR, reply);
    return;
  }

  if (type == MSG_STATUS) {
    if (!can_command(1U, update_session, 0U, 1000U, reply)) { result[0] = GATEWAY_CAN_TIMEOUT; return; }
    set_result(result, reply[1] == 0U ? GATEWAY_OK : GATEWAY_NODE_ERROR, reply);
    return;
  }

  if (type == MSG_REBOOT) {
    if (!can_command(7U, update_session, 0U, 1000U, reply)) { result[0] = GATEWAY_CAN_TIMEOUT; return; }
    set_result(result, reply[1] == 0U ? GATEWAY_OK : GATEWAY_NODE_ERROR, reply);
    *reset_after_reply = result[0] == GATEWAY_OK;
    return;
  }

  result[0] = GATEWAY_FRAME_ERROR;
}

void fw_update_gateway_process(void)
{
  if (!fw_gateway_active) return;
  if (!uart_frame_ready) {
    if (uart_rx_index != 0U && HAL_GetTick() - uart_last_byte_tick > 250U) reset_uart_parser();
    return;
  }

  __DMB();
  const uint8_t type = uart_rx_buffer[5];
  const uint16_t sequence = load_u16(&uart_rx_buffer[6]);
  const uint16_t payload_length = load_u16(&uart_rx_buffer[8]);
  const uint32_t expected_crc = load_u32(&uart_rx_buffer[UART_HEADER_SIZE + payload_length]);
  if (uart_rx_overflow || crc32c(uart_rx_buffer, UART_HEADER_SIZE + payload_length) != expected_crc) {
    uart_frame_ready = false;
    reset_uart_parser();
    return;
  }

  if (last_response_valid && sequence == last_sequence) {
    uart_frame_ready = false;
    reset_uart_parser();
    send_uart_response(last_response_type, sequence, last_response_payload, last_response_length);
    return;
  }

  uint8_t result[8];
  bool reset_after_reply;
  if (last_response_valid && sequence != (uint16_t)(last_sequence + 1U)) {
    set_result(result, GATEWAY_SEQUENCE_ERROR, NULL);
    store_u32(&result[4], (uint32_t)(uint16_t)(last_sequence + 1U));
    reset_after_reply = false;
  } else {
    handle_request(type, &uart_rx_buffer[UART_HEADER_SIZE], payload_length, result, &reset_after_reply);
  }

  last_sequence = sequence;
  last_response_type = (uint8_t)(type | 0x80U);
  last_response_length = sizeof(result);
  memcpy(last_response_payload, result, sizeof(result));
  last_response_valid = true;
  uart_frame_ready = false;
  reset_uart_parser();
  send_uart_response(last_response_type, sequence, result, sizeof(result));
  if (reset_after_reply) {
    HAL_Delay(50U);
    NVIC_SystemReset();
  }
}
