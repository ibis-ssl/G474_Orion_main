/* CM4の更新要求を解析し、CAN1/CAN2上の指定ノードへ896-byte単位で並列配信する。 */
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
#define UART_RX_QUEUE_SIZE 2048U
#define UART_RX_QUEUE_MASK (UART_RX_QUEUE_SIZE - 1U)

#define MSG_ENTER 1U
#define MSG_BEGIN 2U
#define MSG_CHUNK 3U
#define MSG_FINALIZE 4U
#define MSG_REBOOT 5U
#define MSG_STATUS 6U
#define MSG_UART_DIAG_RESET 0x41U

#define GATEWAY_OK 0U
#define GATEWAY_FRAME_ERROR 1U
#define GATEWAY_CAN_TIMEOUT 2U
#define GATEWAY_NODE_ERROR 3U
#define GATEWAY_RANGE_ERROR 4U
#define GATEWAY_SEQUENCE_ERROR 5U

#define CAN_COMMAND_ID 0x610U
#define CAN_DATA_ID_BASE 0x480U
#define CAN_RESPONSE_BASE 0x650U
#define POWER_NODE_ID 100U
#define POWER_STATUS_ID 0x244U
#define NODE_UNUSED 0xFFU
#define CHUNK_CAPACITY 896U

extern volatile bool fw_gateway_active;

static uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
/* USART2 IRQをproducer、main loopをconsumerとするSPSCリング。
 * 2のべき乗サイズとDMBにより、コピー完了前のhead公開を防ぐ。 */
static uint8_t uart_rx_queue[UART_RX_QUEUE_SIZE];
static volatile uint16_t uart_rx_queue_head;
static volatile uint16_t uart_rx_queue_tail;
static volatile uint16_t uart_rx_index;
static volatile uint16_t uart_rx_expected;
static volatile bool uart_frame_ready;
static volatile bool uart_rx_overflow;
static volatile uint32_t uart_last_byte_tick;

/* CM4 UART安定性試験で受信解析と応答送信を切り分ける診断カウンタ。 */
volatile uint32_t fw_uart_frame_count;
volatile uint32_t fw_uart_crc_error_count;
volatile uint32_t fw_uart_ready_overflow_count;
volatile uint32_t fw_uart_response_ok_count;
volatile uint32_t fw_uart_response_fail_count;
volatile uint32_t fw_uart_header_version_error_count;
volatile uint32_t fw_uart_header_length_error_count;
volatile uint32_t fw_uart_header_crc_error_count;
volatile uint8_t fw_uart_last_bad_header[UART_HEADER_SIZE];
volatile uint32_t fw_uart_input_byte_count;
volatile uint32_t fw_uart_queue_overflow_count;
volatile uint32_t fw_uart_raw_hash = UINT32_C(5381);
volatile uint32_t fw_uart_raw_count;
volatile uint32_t fw_uart_queue_hash = UINT32_C(5381);
volatile uint32_t fw_uart_queue_count;
volatile uint32_t fw_uart_magic_start_count;
volatile uint32_t fw_uart_header_ok_count;
volatile uint32_t fw_uart_parser_timeout_count;

static volatile uint8_t can_reply[2][8];
static volatile uint32_t can_reply_counter[2];
static volatile uint8_t power_status[2][8];
static volatile uint32_t power_status_counter[2];
static uint8_t update_session;
static uint8_t update_nodes[2] = {4U, NODE_UNUSED};

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
  uart_rx_queue_head = 0U;
  uart_rx_queue_tail = 0U;
  reset_uart_parser();
}

void fw_update_gateway_uart_rx_byte(uint8_t value)
{
  const uint16_t head = uart_rx_queue_head;
  const uint16_t next = (uint16_t)((head + 1U) & UART_RX_QUEUE_MASK);
  fw_uart_input_byte_count++;
  fw_uart_raw_hash = (fw_uart_raw_hash * UINT32_C(33)) ^ value;
  fw_uart_raw_count++;
  uart_last_byte_tick = HAL_GetTick();
  if (next == uart_rx_queue_tail) {
    fw_uart_queue_overflow_count++;
    return;
  }
  uart_rx_queue[head] = value;
  __DMB();
  uart_rx_queue_head = next;
}

static void parse_uart_byte(uint8_t value)
{
  static const uint8_t magic[4] = {UART_MAGIC_0, UART_MAGIC_1, UART_MAGIC_2, UART_MAGIC_3};
  if (uart_frame_ready) {
    uart_rx_overflow = true;
    fw_uart_ready_overflow_count++;
    return;
  }

  if (uart_rx_index < 4U) {
    if (value == magic[uart_rx_index]) {
      if (uart_rx_index == 0U) fw_uart_magic_start_count++;
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
    const bool version_error = uart_rx_buffer[4] != UART_VERSION;
    const bool length_error = payload_length > UART_MAX_PAYLOAD;
    const bool header_crc_error = crc16_ccitt(uart_rx_buffer, 10U) != load_u16(&uart_rx_buffer[10]);
    if (version_error || length_error || header_crc_error) {
      if (version_error) fw_uart_header_version_error_count++;
      if (length_error) fw_uart_header_length_error_count++;
      if (header_crc_error) fw_uart_header_crc_error_count++;
      memcpy((void *)fw_uart_last_bad_header, uart_rx_buffer, UART_HEADER_SIZE);
      reset_uart_parser();
      return;
    }
    uart_rx_expected = (uint16_t)(UART_HEADER_SIZE + payload_length + UART_TRAILER_SIZE);
    fw_uart_header_ok_count++;
  }

  if (uart_rx_expected != 0U && uart_rx_index == uart_rx_expected) {
    __DMB();
    uart_frame_ready = true;
    fw_uart_frame_count++;
  }
}

bool fw_update_gateway_can_rx(FDCAN_HandleTypeDef * hfdcan, uint32_t identifier, const uint8_t data[8])
{
  uint32_t bus;
  if (!fw_gateway_active) return false;
  if (hfdcan->Instance == FDCAN1) bus = 0U;
  else if (hfdcan->Instance == FDCAN2) bus = 1U;
  else return false;
  /* Powerの実配線バスを安全停止応答から自動判別するため、選択前から両バスで監視する。 */
  if (identifier == POWER_STATUS_ID) {
    memcpy((void *)power_status[bus], data, 8U);
    __DMB();
    power_status_counter[bus]++;
    return true;
  }
  if (update_nodes[bus] == NODE_UNUSED || identifier != CAN_RESPONSE_BASE + update_nodes[bus]) return false;
  memcpy((void *)can_reply[bus], data, 8U);
  __DMB();
  can_reply_counter[bus]++;
  return true;
}

static FDCAN_HandleTypeDef * bus_handle(uint32_t bus)
{
  return bus == 0U ? &hfdcan1 : &hfdcan2;
}

static bool fdcan_send_wait(uint32_t bus, uint32_t identifier, const uint8_t data[8], uint32_t timeout_ms)
{
  FDCAN_HandleTypeDef * hfdcan = bus_handle(bus);
  const uint32_t start = HAL_GetTick();
  while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U) {
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
  return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, data) == HAL_OK;
}

static bool power_safe_stop(void)
{
  uint32_t selected_bus = 2U;
  for (uint32_t bus = 0U; bus < 2U; bus++) {
    if (update_nodes[bus] == POWER_NODE_ID) selected_bus = bus;
  }
  if (selected_bus >= 2U) return true;
  /* 通常の目標電圧コマンドで受理される最小値20.0 Vへ下げる。 */
  const uint8_t target_safe[8] = {0U, 0U, 0U, 0U, 0x00U, 0x00U, 0xA0U, 0x41U};
  const uint8_t charge_disable[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  const uint8_t output_disable[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint32_t before[2] = {power_status_counter[0], power_status_counter[1]};
  uint32_t safe_samples[2] = {0U, 0U};
  for (uint32_t attempt = 0U; attempt < 5U; attempt++) {
    for (uint32_t bus = 0U; bus < 2U; bus++) {
      (void)fdcan_send_wait(bus, 0x110U, target_safe, 100U);
      (void)fdcan_send_wait(bus, 0x110U, charge_disable, 100U);
      (void)fdcan_send_wait(bus, 0x010U, output_disable, 100U);
    }
    const uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < 250U) {
      for (uint32_t bus = 0U; bus < 2U; bus++) {
        if (power_status_counter[bus] == before[bus]) continue;
        __DMB();
        before[bus] = power_status_counter[bus];
        const uint8_t flags = power_status[bus][0];
        safe_samples[bus] = flags == 0U ? safe_samples[bus] + 1U : 0U;
        if (safe_samples[bus] >= 3U) {
          if (bus != selected_bus) {
            update_nodes[selected_bus] = NODE_UNUSED;
            update_nodes[bus] = POWER_NODE_ID;
          }
          return true;
        }
      }
    }
  }
  return false;
}

static bool can_command(uint8_t command, uint8_t token, uint32_t value, uint32_t timeout_ms, uint8_t reply[8])
{
  for (uint32_t attempt = 0U; attempt < 5U; attempt++) {
    uint32_t before[2] = {can_reply_counter[0], can_reply_counter[1]};
    bool sent = true;
    for (uint32_t bus = 0U; bus < 2U; bus++) {
      if (update_nodes[bus] == NODE_UNUSED) continue;
      const uint8_t request[8] = {command, update_nodes[bus], token, 0U, (uint8_t)value, (uint8_t)(value >> 8U), (uint8_t)(value >> 16U), (uint8_t)(value >> 24U)};
      if (!fdcan_send_wait(bus, CAN_COMMAND_ID, request, 100U)) sent = false;
    }
    if (!sent) continue;
    const uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout_ms) {
      bool complete = true;
      for (uint32_t bus = 0U; bus < 2U; bus++) if (update_nodes[bus] != NODE_UNUSED && can_reply_counter[bus] == before[bus]) complete = false;
      if (complete) {
        __DMB();
        bool first = true;
        uint8_t reference[8] = {0};
        for (uint32_t bus = 0U; bus < 2U; bus++) {
          if (update_nodes[bus] == NODE_UNUSED) continue;
          uint8_t current[8]; memcpy(current, (const void *)can_reply[bus], 8U);
          if (current[0] != (uint8_t)(command | 0x80U) || current[2] != update_nodes[bus] || current[3] != token) { complete = false; break; }
          if (first) { memcpy(reference, current, 8U); memcpy(reply, current, 8U); first = false; }
          else if (current[1] != reference[1]) { memcpy(reply, current, 8U); reply[1] = 3U; }
        }
        if (complete) return true;
      }
    }
  }
  return false;
}

static bool can_command_bus(uint32_t bus, uint8_t command, uint8_t token, uint32_t value, uint32_t timeout_ms, uint8_t reply[8])
{
  if (bus >= 2U || update_nodes[bus] == NODE_UNUSED) return false;
  for (uint32_t attempt = 0U; attempt < 5U; attempt++) {
    uint32_t before = can_reply_counter[bus];
    const uint8_t request[8] = {command, update_nodes[bus], token, 0U, (uint8_t)value, (uint8_t)(value >> 8U), (uint8_t)(value >> 16U), (uint8_t)(value >> 24U)};
    if (!fdcan_send_wait(bus, CAN_COMMAND_ID, request, 100U)) continue;
    const uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout_ms) {
      if (can_reply_counter[bus] == before) continue;
      __DMB();
      uint8_t current[8];
      memcpy(current, (const void *)can_reply[bus], 8U);
      before = can_reply_counter[bus];
      if (current[0] == (uint8_t)(command | 0x80U) && current[2] == update_nodes[bus] && current[3] == token) {
        memcpy(reply, current, 8U);
        return true;
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
        HAL_UART_Transmit(&huart2, frame, UART_HEADER_SIZE + payload_length + UART_TRAILER_SIZE, 100U) == HAL_OK) {
      fw_uart_response_ok_count++;
      return;
    }
  }
  fw_uart_response_fail_count++;
}

static void set_result(uint8_t payload[8], uint8_t gateway_status, const uint8_t node_reply[8])
{
  payload[0] = gateway_status;
  payload[1] = node_reply != NULL ? node_reply[1] : 0U;
  payload[2] = node_reply != NULL ? node_reply[2] : (update_nodes[0] != NODE_UNUSED ? update_nodes[0] : update_nodes[1]);
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
    update_nodes[0] = payload[1];
    update_nodes[1] = payload[2];
    if (update_nodes[0] == NODE_UNUSED && update_nodes[1] == NODE_UNUSED) { result[0] = GATEWAY_RANGE_ERROR; return; }
    if (!power_safe_stop()) { result[0] = GATEWAY_NODE_ERROR; result[1] = 1U; result[2] = POWER_NODE_ID; return; }
    for (uint32_t i = 0U; i < 10U; i++) {
      for (uint32_t bus = 0U; bus < 2U; bus++) {
        if (update_nodes[bus] == NODE_UNUSED) continue;
        const uint8_t enter[8] = {'O', 'F', 'W', 'U', 'P', update_nodes[bus], 0U, update_session};
        (void)fdcan_send_wait(bus, 0x600U, enter, 100U);
      }
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
    uint8_t pending_mask = 0U;
    bool received_node_reply = false;
    for (uint32_t bus = 0U; bus < 2U; bus++) if (update_nodes[bus] != NODE_UNUSED) pending_mask |= (uint8_t)(1U << bus);
    for (uint32_t chunk_attempt = 0U; chunk_attempt < 3U && pending_mask != 0U; chunk_attempt++) {
      uint8_t active_mask = 0U;
      for (uint32_t bus = 0U; bus < 2U; bus++) {
        const uint8_t bus_mask = (uint8_t)(1U << bus);
        if ((pending_mask & bus_mask) == 0U) continue;
        uint8_t bus_reply[8];
        if (!can_command_bus(bus, 4U, token, offset, 500U, bus_reply)) continue;
        received_node_reply = true;
        memcpy(reply, bus_reply, 8U);
        if (bus_reply[1] == 0U) active_mask |= bus_mask;
        else if (bus_reply[1] == 2U && load_u32(&bus_reply[4]) == offset + chunk_length) pending_mask &= (uint8_t)~bus_mask;
        else { set_result(result, GATEWAY_NODE_ERROR, bus_reply); return; }
      }
      if (active_mask == 0U) continue;
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
        for (uint32_t bus = 0U; bus < 2U; bus++) {
          if ((active_mask & (uint8_t)(1U << bus)) != 0U && !fdcan_send_wait(bus, CAN_DATA_ID_BASE + sequence, data, 100U)) { send_ok = false; break; }
        }
        if (!send_ok) break;
        if (chunk_attempt == 0U && (injection & 2U) != 0U && sequence == 7U) {
          for (uint32_t bus = 0U; bus < 2U; bus++) if ((active_mask & (uint8_t)(1U << bus)) != 0U && !fdcan_send_wait(bus, CAN_DATA_ID_BASE + sequence, data, 100U)) send_ok = false;
          if (!send_ok) break;
        }
      }
      if (!send_ok) continue;
      HAL_Delay(2U);
      for (uint32_t bus = 0U; bus < 2U; bus++) {
        const uint8_t bus_mask = (uint8_t)(1U << bus);
        if ((active_mask & bus_mask) == 0U) continue;
        uint8_t bus_reply[8];
        bool completed = false;
        if (can_command_bus(bus, 5U, token, chunk_crc, 500U, bus_reply)) {
          received_node_reply = true;
          memcpy(reply, bus_reply, 8U);
          if (bus_reply[1] == 0U) completed = true;
          else if (bus_reply[1] != 3U && bus_reply[1] != 4U) { set_result(result, GATEWAY_NODE_ERROR, bus_reply); return; }
        } else if (can_command_bus(bus, 1U, token, 0U, 250U, bus_reply)) {
          received_node_reply = true;
          memcpy(reply, bus_reply, 8U);
          const uint32_t confirmed = load_u32(&bus_reply[4]);
          if (bus_reply[1] == 0U && confirmed == offset + chunk_length) completed = true;
          else if (bus_reply[1] != 0U || confirmed != offset) { set_result(result, GATEWAY_NODE_ERROR, bus_reply); return; }
        }
        if (completed) pending_mask &= (uint8_t)~bus_mask;
      }
    }
    if (pending_mask == 0U) { set_result(result, GATEWAY_OK, reply); result[1] = 0U; return; }
    if (received_node_reply) set_result(result, GATEWAY_NODE_ERROR, reply);
    else result[0] = GATEWAY_CAN_TIMEOUT;
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
    if (length > 1U) { result[0] = GATEWAY_RANGE_ERROR; return; }
    if (!can_command(7U, update_session, 0U, 1000U, reply)) { result[0] = GATEWAY_CAN_TIMEOUT; return; }
    set_result(result, reply[1] == 0U ? GATEWAY_OK : GATEWAY_NODE_ERROR, reply);
    *reset_after_reply = result[0] == GATEWAY_OK && (length == 0U || payload[0] == 0U);
    return;
  }

  if (type == MSG_UART_DIAG_RESET) {
    if (length != 0U) { result[0] = GATEWAY_RANGE_ERROR; return; }
    fw_uart_raw_hash = UINT32_C(5381);
    fw_uart_raw_count = 0U;
    fw_uart_queue_hash = UINT32_C(5381);
    fw_uart_queue_count = 0U;
    return;
  }

  result[0] = GATEWAY_FRAME_ERROR;
}

void fw_update_gateway_process(void)
{
  if (!fw_gateway_active) return;
  while (!uart_frame_ready) {
    const uint16_t tail = uart_rx_queue_tail;
    if (tail == uart_rx_queue_head) break;
    __DMB();
    const uint8_t value = uart_rx_queue[tail];
    uart_rx_queue_tail = (uint16_t)((tail + 1U) & UART_RX_QUEUE_MASK);
    fw_uart_queue_hash = (fw_uart_queue_hash * UINT32_C(33)) ^ value;
    fw_uart_queue_count++;
    parse_uart_byte(value);
  }
  if (!uart_frame_ready) {
    /* HAL_GetTick()取得直後にIRQがlast_byte_tickを更新すると、符号なし減算が
     * underflowして偽timeoutになる。snapshotの一致を再確認してから破棄する。 */
    const uint32_t last_byte_tick = uart_last_byte_tick;
    const uint32_t now = HAL_GetTick();
    if (uart_rx_index != 0U && now - last_byte_tick > 250U && last_byte_tick == uart_last_byte_tick) {
      fw_uart_parser_timeout_count++;
      reset_uart_parser();
    }
    return;
  }

  __DMB();
  const uint8_t type = uart_rx_buffer[5];
  const uint16_t sequence = load_u16(&uart_rx_buffer[6]);
  const uint16_t payload_length = load_u16(&uart_rx_buffer[8]);
  const uint32_t expected_crc = load_u32(&uart_rx_buffer[UART_HEADER_SIZE + payload_length]);
  if (uart_rx_overflow || crc32c(uart_rx_buffer, UART_HEADER_SIZE + payload_length) != expected_crc) {
    fw_uart_crc_error_count++;
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
