#include "LinProtocolHandler.h"
// Third-Party Library Headers
#include "LinSerial.h"  // Explicit include for LinSerial
#include "LinCommon.h"  // Explicit include for shared constants
#include "esp_log.h"    // Explicit include for ESP_LOG* macros
#include "esp_timer.h"  // Explicit include for esp_timer_get_time()
#include <cstring>      // For memset

#include "driver/uart.h"

// Add low-level UART register definitions (needed for uart_dev_t)
extern "C" {
#include "soc/uart_struct.h"  // defines uart_dev_t, UART0, UART1 (ESP32-C3 has only 2 UARTs)
#include "hal/uart_ll.h"
#include "esp_intr_alloc.h"
}

#include <cstring>

// Helper to map uart_port_t to hw struct (remove unsupported UART2 for C3)
static inline IRAM_ATTR uart_dev_t *lin_get_dev(uart_port_t n) {
  switch (n) {
    case UART_NUM_0:
      return &UART0;
    case UART_NUM_1:
      return &UART1;
    default:
      return nullptr;
  }
}

static const char *TAG = "LinProt";

LinProtocolHandler::LinProtocolHandler(LinSerial *serial, int baud_rate, lin_schedule_item_t *schedule,
                                       lin_discovered_id_t *discoveredIDs, lin_respond_data_t *responses)
    : serial_(serial),
      baud_rate_(baud_rate),
      schedule_(schedule),
      responses_(responses),
      discoveredIDs_(discoveredIDs),
      state_(READ_STATE_BREAK) {
  byte_time_us_ = 1000000 / baud_rate_ * 10;  // 10 bits per byte at 19200 baud ≈ 520us

  // Initialize statistics
  resetStatistics();
}

// Destructor to free buffer
LinProtocolHandler::~LinProtocolHandler() { ; }

void LinProtocolHandler::onID(uint8_t id) {
  if (!serial_)
    return;
  for (size_t i = 0; i < LIN_MAX_RESPONSES; ++i) {
    if (responses_[i].id == id && responses_[i].dataLength) {
      serial_->writeBytes(responses_[i].responseData, responses_[i].dataLength);

      // Update statistics - ID request answered
      statistics_.id_requests_answered++;
      statistics_.total_bus_bytes += responses_[i].dataLength;
      return;
    }
  }
}

void LinProtocolHandler::onData(uint8_t pid, const uint8_t *data, size_t len) {
  ESP_LOGD(TAG, "Data received for PID 0x%02X, length %zu", pid, len);
  if (len == 0 || len > 16)
    return;

  // Update statistics - data received
  statistics_.data_bytes_received += len;
  statistics_.total_bus_bytes += len;

  // Track unique IDs
  uint8_t id = pid & 0x3F;
  if (!(unique_id_tracker_[id / 8] & (1 << (id % 8)))) {
    unique_id_tracker_[id / 8] |= (1 << (id % 8));
    statistics_.unique_ids_seen++;
  }

  uint8_t expectedChecksum = calculateChecksum(pid, data, len - 1);
  if (data[len - 1] != expectedChecksum) {
    ESP_LOGD(TAG, "Checksum error for PID 0x%02X: got 0x%02X, expected 0x%02X", pid, data[len - 1], expectedChecksum);

    // Update statistics - checksum error
    statistics_.checksum_errors++;
    return;
  }

  // id already declared above - reuse it
  int oldest_idx = -1;
  int64_t oldest_time = INT64_MAX;

  // Scan for existing entry (match by ID) or first free (dataLength == 0)
  for (size_t i = 0; i < LIN_MAX_DISCOVERED_PIDS; ++i) {
    if (discoveredIDs_[i].dataLength > 0) {
      // Used slot
      if (discoveredIDs_[i].id == id) {
        size_t copyLen = (len - 1 > LIN_MAX_DATA_SIZE) ? LIN_MAX_DATA_SIZE : (len - 1);
        memcpy(discoveredIDs_[i].data, data, copyLen);
        discoveredIDs_[i].dataLength = copyLen;
        discoveredIDs_[i].updateCount++;
        discoveredIDs_[i].lastUpdateTime = esp_timer_get_time();
        ESP_LOGD(TAG, "Updated discovered ID 0x%02X", id);
        return;
      }
      if (discoveredIDs_[i].lastUpdateTime < oldest_time) {
        oldest_time = discoveredIDs_[i].lastUpdateTime;
        oldest_idx = (int) i;
      }
    } else {
      // Free slot (dataLength == 0). Use it.
      discoveredIDs_[i].id = id;
      size_t copyLen = (len - 1 > LIN_MAX_DATA_SIZE) ? LIN_MAX_DATA_SIZE : (len - 1);
      memcpy(discoveredIDs_[i].data, data, copyLen);
      discoveredIDs_[i].dataLength = copyLen;
      discoveredIDs_[i].updateCount = 1;
      discoveredIDs_[i].lastUpdateTime = esp_timer_get_time();
      ESP_LOGI(TAG, "Discovered new ID 0x%02X", id);
      return;
    }
  }

  // All slots used – overwrite oldest
  if (oldest_idx >= 0) {
    size_t copyLen = (len - 1 > LIN_MAX_DATA_SIZE) ? LIN_MAX_DATA_SIZE : (len - 1);
    discoveredIDs_[oldest_idx].id = id;
    memcpy(discoveredIDs_[oldest_idx].data, data, copyLen);
    discoveredIDs_[oldest_idx].dataLength = copyLen;
    discoveredIDs_[oldest_idx].updateCount = 1;
    discoveredIDs_[oldest_idx].lastUpdateTime = esp_timer_get_time();
    ESP_LOGD(TAG, "Overwrote oldest slot with ID 0x%02X", id);
  }
}

uint8_t IRAM_ATTR LinProtocolHandler::master_send() {
  int32_t current_time_ms = (int32_t) (esp_timer_get_time() / 1000);
  for (size_t i = 0; i < LIN_MAX_ID_RQ_COUNT; i++) {
    if (schedule_[i].updateInterval == 0)
      continue;  // free slot sentinel

    int64_t elapsed_ms = current_time_ms - schedule_[i].last_send_time_ms;
    if (schedule_[i].last_send_time_ms == 0 || elapsed_ms >= schedule_[i].updateInterval) {
      if (!isBusIdle(3))
        return 0;

      uint8_t id = schedule_[i].id;
      if (serial_) {
        serial_->sendBreak();
        uint8_t sync = 0x55;
        serial_->writeBytes(&sync, 1);
        uint8_t pid = calculate_pid(id);
        serial_->writeBytes(&pid, 1);

        // Update statistics - master request sent
        statistics_.master_requests_sent++;
        statistics_.total_bus_bytes += 3;  // break + sync + pid
      } else {
        // Update statistics - master send failure
        statistics_.master_send_failures++;
      }
      schedule_[i].last_send_time_ms = current_time_ms;
      return id;
    }
  }
  return 0;
}

// FIX: Treat argument as PID (with parity) OR raw ID. Always mask to 6-bit ID
// and only include PID in checksum for enhanced frames (non-diagnostic).
uint8_t LinProtocolHandler::calculateChecksum(uint8_t pid_or_id, const uint8_t *data, size_t len) {
  uint8_t id = pid_or_id & 0x3F;  // 6-bit ID
  uint16_t sum = 0;

  // Enhanced checksum (LIN 2.x): include protected identifier (with parity)
  // Classic checksum: data bytes only.
  // Diagnostic frames (0x3C, 0x3D) ALWAYS use classic checksum.
  if (id != 0x3C && id != 0x3D) {
    // Use canonical PID value (rebuild from ID) to avoid parity inconsistencies
    uint8_t pid = calculate_pid(id);
    sum += pid;
  }

  for (size_t i = 0; i < len; ++i) {
    sum += data[i];
  }

  // Fold carries
  while (sum > 0xFF) {
    sum = (sum & 0xFF) + (sum >> 8);
  }
  return (uint8_t) (~sum);
}

// Get data for a specific PID, returns length or 0 if not found
size_t LinProtocolHandler::getDataForID(uint8_t id, uint8_t *dataBuf) {
  for (size_t i = 0; i < LIN_MAX_DISCOVERED_PIDS; ++i) {
    if (discoveredIDs_[i].dataLength == 0)
      continue;  // skip free
    if (discoveredIDs_[i].id == id) {
      memcpy(dataBuf, discoveredIDs_[i].data, discoveredIDs_[i].dataLength);
      return discoveredIDs_[i].dataLength;
    }
  }
  return 0;
}

void inline LinProtocolHandler::resetDecodeState() { state_ = READ_STATE_BREAK; }

void LinProtocolHandler::onInterByteTimeout() {
  if (state_ == READ_STATE_DATA) {
    // If we already have at least 2 bytes (payload+checksum min) finalize
    if (currentDataCount_ >= 2) {
      onData(currentPIDWithParity_, currentData_, currentDataCount_);
    }
    resetDecodeState();
  }
}

IRAM_ATTR lin_read_state_t LinProtocolHandler::receiveDecode(uint8_t readbyte, uint8_t command) {
  // this is called by ISR
  // called by ISR with new serial data (command = 0), and by ISR timout (command = 1).
  // must be fast as hell, to keep LIN timings
  // straight fall through state machine, no logging, no time keeping until ID reply (first data byte)

  if (command == 0) {
    last_bus_activity_us_ = esp_timer_get_time();
  }

  switch (state_) {
    case READ_STATE_BREAK:
      currentDataCount_ = 0;   // init for later data receive
      if (readbyte == 0x00) {  // LIN_BREAK
        state_ = READ_STATE_SYNC;
      }
      break;
    case READ_STATE_SYNC:
      if (readbyte == 0x55) {  // LIN_SYNC
        state_ = READ_STATE_SID;
      } else {
        state_ = READ_STATE_BREAK;
      }
      break;
    case READ_STATE_SID: {
      // Accept any PID (0..63 with parity)
      uint8_t pid = readbyte;
      uint8_t id = pid & 0x3F;
      if (calculate_pid(id) == pid) {
        currentPIDWithParity_ = pid;
        currentID_ = id;

        // Immediately transmit response (first data byte timing critical)
        onID(id);

        // log received ID req
        ESP_LOGD(TAG, "RX ID req: 0x%02X", id);

        // Enter data phase to capture response (including our own echo HW loops back)
        currentDataCount_ = 0;
        timeout_LinRecieive_ = esp_timer_get_time() + (300 * (byte_time_us_));
        state_ = READ_STATE_DATA;
      } else {
        // Parity error -> reset
        statistics_.pid_errors++;
        state_ = READ_STATE_BREAK;
      }
      break;
    }

    case READ_STATE_DATA:
      if (command == 0) {  // only real bytes
        if (currentDataCount_ < (LIN_MAX_DATA_SIZE + 1)) {
          currentData_[currentDataCount_++] = readbyte;
          // Update absolute timeout (fallback safety)
          timeout_LinRecieive_ = esp_timer_get_time() + byte_time_us_;
        } else {
          // Overflow protection: finalize
          if (currentDataCount_ >= 2) {
            onData(currentPIDWithParity_, currentData_, currentDataCount_);
          }
          resetDecodeState();
        }
      }
      break;

    default:
      resetDecodeState();
      break;
  }
  return state_;
}

// Helper: check bus idle (driver-only)
bool LinProtocolHandler::isBusIdle(uint32_t required_idle_bits) {
  if (required_idle_bits == 0)
    required_idle_bits = 3;
  int64_t now = esp_timer_get_time();
  int64_t bit_time_us = (baud_rate_ > 0) ? (1000000LL / baud_rate_) : 52;
  int64_t needed_us = (int64_t) required_idle_bits * bit_time_us;

  if (state_ == READ_STATE_DATA)
    return false;
  if ((now - last_bus_activity_us_) < needed_us)
    return false;

  // HW TX idle check (shift register + FIFO empty)
  uart_dev_t *hw = lin_get_dev(serial_->getPort());
  if (hw && !uart_ll_is_tx_idle(hw))
    return false;

  return true;
}

// Utility functions remain unchanged
uint8_t LinProtocolHandler::calculate_pid(uint8_t id) {
  uint8_t p0 = ((id >> 0) ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 0x01;
  uint8_t p1 = ~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5)) & 0x01;
  uint8_t pid = id | (p0 << 6) | (p1 << 7);
  return pid;
}

uint8_t LinProtocolHandler::get_id_from_pid(uint8_t pid) {
  uint8_t id = pid & 0x3F;
  if (calculate_pid(id) == pid) {
    return id;
  } else {
    return 0x3F;  // Invalid PID
  }
}

// Statistics implementation
void LinProtocolHandler::resetStatistics() {
  memset(&statistics_, 0, sizeof(statistics_));
  statistics_.stats_start_time = esp_timer_get_time();
}

void LinProtocolHandler::updateStatistics() {
  int64_t now = esp_timer_get_time();
  int64_t window_duration_us = now - statistics_.stats_start_time;

  // Update every 10 seconds (10,000,000 microseconds)
  if (window_duration_us >= 10000000) {
    // Debug: Show raw counters before calculation
    ESP_LOGD(TAG, "Stats window: master_reqs=%u, id_answers=%u, data_bytes=%u, errors=%u+%u+%u",
             statistics_.master_requests_sent, statistics_.id_requests_answered, statistics_.data_bytes_received,
             statistics_.master_send_failures, statistics_.pid_errors, statistics_.checksum_errors);

    // Calculate rates (per second)
    double window_duration_s = window_duration_us / 1000000.0;

    statistics_.master_reqs_per_sec = statistics_.master_requests_sent / window_duration_s;
    statistics_.id_answers_per_sec = statistics_.id_requests_answered / window_duration_s;
    statistics_.data_bytes_per_sec = statistics_.data_bytes_received / window_duration_s;

    // Calculate bus load percentage
    // Assuming 8N1 format: 1 start + 8 data + 1 stop = 10 bits per byte
    int64_t theoretical_max_bits = (int64_t) baud_rate_ * window_duration_s;
    int64_t actual_bits = statistics_.total_bus_bytes * 10;
    statistics_.estimated_bus_load_percent =
        (theoretical_max_bits > 0) ? (double) actual_bits / theoretical_max_bits * 100.0 : 0.0;

    // Calculate error rates
    int64_t total_transactions = statistics_.master_requests_sent + statistics_.id_requests_answered;
    int64_t total_errors = statistics_.master_send_failures + statistics_.pid_errors + statistics_.checksum_errors;
    statistics_.error_rate_percent =
        (total_transactions > 0) ? (double) total_errors / total_transactions * 100.0 : 0.0;

    // Reset for next window
    statistics_.stats_start_time = now;
    statistics_.master_requests_sent = 0;
    statistics_.id_requests_answered = 0;
    statistics_.data_bytes_received = 0;
    statistics_.master_send_failures = 0;
    statistics_.pid_errors = 0;
    statistics_.checksum_errors = 0;
    statistics_.total_bus_bytes = 0;
  }
}

void LinProtocolHandler::logStatistics() const {
  ESP_LOGI(TAG, "LIN Statistics (10s window):");
  ESP_LOGI(TAG, "  Master requests: %.1f/s", statistics_.master_reqs_per_sec);
  ESP_LOGI(TAG, "  ID responses: %.1f/s", statistics_.id_answers_per_sec);
  ESP_LOGI(TAG, "  Data bytes: %.1f/s", statistics_.data_bytes_per_sec);
  ESP_LOGI(TAG, "  Unique IDs seen: %u", statistics_.unique_ids_seen);
  ESP_LOGI(TAG, "  Bus load: %.1f%%", statistics_.estimated_bus_load_percent);
  ESP_LOGI(TAG, "  Error rate: %.1f%%", statistics_.error_rate_percent);
  ESP_LOGI(TAG, "  Stats window: %lld ms", (esp_timer_get_time() - statistics_.stats_start_time) / 1000);
}
