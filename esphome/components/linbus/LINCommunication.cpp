#include "LINCommunication.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

// Project Headers
#include "LinSerial.h"
#include "LinProtocolHandler.h"

static const char *TAG = "LIN_COMM";

// Add IRAM callback wrapper that forwards bytes to LinProtocolHandler::receiveDecode
static IRAM_ATTR void lin_serial_rx_cb(void *ctx, uint8_t b) {
  if (!ctx)
    return;
  LinProtocolHandler *ph = reinterpret_cast<LinProtocolHandler *>(ctx);
  // call ISR-ready receiveDecode (signature: receiveDecode(uint8_t, uint8_t))
  ph->receiveDecode(b, 0);
}

LINCommunication::LINCommunication(uart_port_t uart_num, int tx_pin, int rx_pin, int cs_pin)
    : uart_num_(uart_num), tx_pin_(tx_pin), rx_pin_(rx_pin), cs_pin_(cs_pin), mode_(LIN_MODE_LISTENER), baud_rate_(0) {
  // Initialize LinSerial (can be nullptr until init)
  lin_serial_ = nullptr;

  // Initialize arrays to zero
  memset(schedule_, 0, sizeof(schedule_));
  memset(responses_, 0, sizeof(responses_));
  memset(discoveredIDs_, 0, sizeof(discoveredIDs_));
}

LINCommunication::~LINCommunication() {
  delete lin_protocol_handler_;
  lin_protocol_handler_ = nullptr;
  delete lin_serial_;
  lin_serial_ = nullptr;
}

esp_err_t LINCommunication::init(lin_mode_t mode, int baud_rate) {
  if (is_initialized_) {
    ESP_LOGW(TAG, "LINCommunication is already initialized. Skipping reinitialization.");
    return ESP_ERR_INVALID_STATE;
  }

  mode_ = mode;
  baud_rate_ = baud_rate;

  // Initialize LinSerial
  lin_serial_ = new LinSerial(uart_num_, tx_pin_, rx_pin_, baud_rate_);

  // Now create LinProtocolHandler with valid pointers
  lin_protocol_handler_ = new LinProtocolHandler(lin_serial_, baud_rate_, schedule_, discoveredIDs_, responses_);

  if (cs_pin_ != -1) {
    // Configure CS  pin
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << (gpio_num_t) cs_pin_);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level((gpio_num_t) cs_pin_, 1);
  }

  if (lin_serial_) {
    BaseType_t ok = xTaskCreatePinnedToCore(uart_event_task_trampoline, "lin_uart_evt", 4096, this,
                                            configMAX_PRIORITIES - 1, &uart_event_task_handle_, tskNO_AFFINITY);
    if (ok != pdPASS) {
      ESP_LOGE(TAG, "Failed to create UART event task");
      return ESP_FAIL;
    }
  }
  // Approx 3 byte times (10 bits/byte). 3 * 10 * 1e6 / baud
  inter_byte_timeout_us_ = (uint32_t) ((3ULL * 10ULL * 1000000ULL) / (uint32_t) baud_rate_);

  ESP_LOGI(TAG, "LINCommunication initialized in mode %d at %d baud", mode_, baud_rate_);
  is_initialized_ = true;  // Mark as initialized

  return ESP_OK;
}

void LINCommunication::uart_event_task_trampoline(void *arg) {
  auto *self = reinterpret_cast<LINCommunication *>(arg);
  if (!self || !self->lin_serial_ || !self->lin_protocol_handler_) {
    vTaskDelete(nullptr);
    return;
  }
  QueueHandle_t q = self->lin_serial_->getEventQueue();
  uart_event_t evt;
  uint8_t buf[32];

  // Convert desired microsecond inter-byte timeout to FreeRTOS ticks (at least 1 tick)
  TickType_t wait_ticks = pdMS_TO_TICKS((self->inter_byte_timeout_us_ + 999) / 1000  // round up to ms
  );
  if (wait_ticks == 0)
    wait_ticks = 1;

  for (;;) {
    // Wait for UART event OR timeout (idle gap)
    if (xQueueReceive(q, &evt, wait_ticks) == pdTRUE) {
      switch (evt.type) {
        case UART_DATA: {
          int r = uart_read_bytes(self->uart_num_, buf, evt.size, 0);
          for (int i = 0; i < r; ++i) {
            self->lin_protocol_handler_->receiveDecode(buf[i], 0);
          }
          // (No manual finalize here; rely on timeout path)
          break;
        }
        case UART_BREAK:
          // Optional: could force state to SYNC wait
          // self->lin_protocol_handler_->forceBreak(); (not implemented)
          break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
          uart_flush_input(self->uart_num_);
          break;
        default:
          break;
      }
    } else {
      // Queue wait timed out: inter-byte gap -> finalize frame if in DATA
      self->lin_protocol_handler_->onInterByteTimeout();
    }
  }
}

bool LINCommunication::lin_process() {
  // store now us:
  int64_t now = esp_timer_get_time();

  // send id if due and store for logging
  uint8_t id = 0;
  id = lin_protocol_handler_->master_send();

  // store time master send took:
  int64_t send_time = esp_timer_get_time() - now;

  // Built‑in self test progression
  if (self_test_enabled_) {
    selfTestTick();
  }

  return true;
}

// master scheduler functions

bool LINCommunication::master_add_id_schedule(uint8_t id, uint32_t interval_ms) {
  // Update existing
  for (size_t i = 0; i < LIN_MAX_ID_RQ_COUNT; i++) {
    if (schedule_[i].updateInterval == 0)
      continue;  // skip free
    if (schedule_[i].id == id) {
      schedule_[i].updateInterval = interval_ms;
      schedule_[i].last_send_time_ms = 0;
      ESP_LOGI(TAG, "Updated schedule ID 0x%02X interval %u ms", id, interval_ms);
      return true;
    }
  }
  // Find free slot
  for (size_t i = 0; i < LIN_MAX_ID_RQ_COUNT; i++) {
    if (schedule_[i].updateInterval == 0) {
      schedule_[i].id = id;
      schedule_[i].updateInterval = interval_ms;
      schedule_[i].last_send_time_ms = 0;
      ESP_LOGI(TAG, "Added schedule ID 0x%02X interval %u ms", id, interval_ms);
      return true;
    }
  }
  ESP_LOGW(TAG, "Schedule full, cannot add ID 0x%02X", id);
  return false;
}

bool LINCommunication::master_remove_id_schedule(uint8_t id) {
  for (size_t i = 0; i < LIN_MAX_ID_RQ_COUNT; i++) {
    if (schedule_[i].updateInterval == 0)
      continue;
    if (schedule_[i].id == id) {
      schedule_[i].updateInterval = 0;  // mark free
      schedule_[i].last_send_time_ms = 0;
      ESP_LOGI(TAG, "Removed schedule ID 0x%02X", id);
      return true;
    }
  }
  ESP_LOGW(TAG, "ID 0x%02X not found in schedule", id);
  return false;
}

// response handling functions
bool LINCommunication::set_response_data(uint8_t id, uint8_t *data, uint8_t data_length) {
  if (id > 0x3F)
    return false;
  if (data_length > LIN_MAX_DATA_SIZE)
    data_length = LIN_MAX_DATA_SIZE;

  // If caller wants to clear: data_length == 0
  if (data_length == 0) {
    for (size_t i = 0; i < LIN_MAX_RESPONSES; ++i) {
      if (responses_[i].dataLength == 0)
        continue;
      if (responses_[i].id == id) {
        responses_[i].dataLength = 0;  // mark free
        // (optional) zero buffer
        memset(responses_[i].responseData, 0, sizeof(responses_[i].responseData));
        return true;
      }
    }
    return true;  // nothing to clear
  }

  // Update existing
  for (size_t i = 0; i < LIN_MAX_RESPONSES; ++i) {
    if (responses_[i].dataLength == 0)
      continue;
    if (responses_[i].id == id) {
      memcpy(responses_[i].responseData, data, data_length);
      // Compute checksum (classic/enhanced handled by handler’s calculateChecksum)
      uint8_t cs = lin_protocol_handler_->calculateChecksum(id, data, data_length);
      responses_[i].responseData[data_length] = cs;
      responses_[i].dataLength = data_length + 1;
      return true;
    }
  }
  // Find free (dataLength == 0)
  for (size_t i = 0; i < LIN_MAX_RESPONSES; ++i) {
    if (responses_[i].dataLength != 0)
      continue;
    responses_[i].id = id;  // keep ID
    memcpy(responses_[i].responseData, data, data_length);
    uint8_t cs = lin_protocol_handler_->calculateChecksum(id, data, data_length);
    responses_[i].responseData[data_length] = cs;
    responses_[i].dataLength = data_length + 1;
    responses_[i].lastResponseTime = 0;
    return true;
  }
  return false;  // no free slot
}

bool LINCommunication::get_response_4id(uint8_t id, lin_discovered_id_t *lindata) {
  if (!lindata) {
    return false;
  }

  // Search for the ID in discoveredIDs_ array
  for (size_t i = 0; i < LIN_MAX_DISCOVERED_PIDS; i++) {
    if (discoveredIDs_[i].dataLength > 0 && discoveredIDs_[i].id == id) {
      // Found the ID, copy data
      *lindata = discoveredIDs_[i];
      return true;
    }
  }

  return false;
}

void LINCommunication::dumpReceivedPIDsToLog() const {
  ESP_LOGI(TAG, "| %-5s | %-4s | %-3s | %-7s | %-8s | %-24s |", "Idx", "ID", "Len", "Count", "Last(ms)", "Data(hex)");
  ESP_LOGI(TAG, "|-------|------|-----|---------|----------|--------------------------|");
  for (size_t i = 0; i < LIN_MAX_DISCOVERED_PIDS; i++) {
    if (discoveredIDs_[i].dataLength == 0)
      continue;  // skip free slot (ID may be 0 validly)

    char dataStr[3 * LIN_MAX_DATA_SIZE + 1] = {0};
    char *p = dataStr;
    for (uint8_t j = 0; j < discoveredIDs_[i].dataLength; ++j) {
      sprintf(p, "%02X ", discoveredIDs_[i].data[j]);
      p += 3;
    }

    ESP_LOGI(TAG, "| %-5zu | 0x%02X | %-3u | %-7u | %-8lld | %-24s |", i, discoveredIDs_[i].id,
             discoveredIDs_[i].dataLength, discoveredIDs_[i].updateCount,
             (esp_timer_get_time() - discoveredIDs_[i].lastUpdateTime) / 1000, dataStr);
  }
}

// TEST stuff

void LINCommunication::enableSelfTest(bool enable) {
  self_test_enabled_ = enable;
  if (!enable) {
    self_test_initialized_ = false;
  }
}

void LINCommunication::selfTestTick() {
  if (mode_ != LIN_MODE_MASTER)
    return;
  if (!lin_protocol_handler_)
    return;
  if (!self_test_initialized_) {
    selfTestInitIfNeeded();
    return;
  }
  for (size_t i = 0; i < SELF_TEST_MAX_IDS; ++i) {
    auto &st = self_test_stats_[i];
    for (size_t d = 0; d < LIN_MAX_DISCOVERED_PIDS; ++d) {
      if (discoveredIDs_[d].dataLength == 0)
        continue;
      if (discoveredIDs_[d].id == st.id) {
        int64_t t = discoveredIDs_[d].lastUpdateTime;
        if (t != st.first_update_time_us) {
          st.seen_updates++;
          if (st.first_update_time_us == 0) {
            st.first_update_time_us = t;
          }
          if (st.prev_update_time_us != 0) {
            int64_t interval = t - st.prev_update_time_us;
            if (st.min_interval_us == 0 || interval < st.min_interval_us)
              st.min_interval_us = interval;
            if (interval > st.max_interval_us)
              st.max_interval_us = interval;
            int64_t expected_us = (int64_t) st.scheduled_interval_ms * 1000;
            if (interval > (expected_us + expected_us / 2)) {
              st.missed_windows++;
            }
          }
          st.prev_update_time_us = t;
          st.last_update_time_us = t;
        }
        break;
      }
    }
  }
  selfTestEvaluate();
}

void LINCommunication::selfTestInitIfNeeded() {
  struct CaseDef {
    uint8_t id;
    uint32_t interval_ms;
    uint8_t payload_len;
  };
  static const CaseDef CASES[] = {{0x00, 120, 3}, {0x01, 150, 2}, {0x15, 180, 4}, {0x1F, 200, 8}, {0x3B, 220, 3},
                                  {0x3C, 250, 2}, {0x3D, 280, 2}, {0x3E, 300, 6}, {0x3F, 330, 1}};
  const size_t case_count = sizeof(CASES) / sizeof(CASES[0]);
  if (case_count > SELF_TEST_MAX_IDS) {
    ESP_LOGW(TAG, "SelfTest truncated: too many cases (%zu)", case_count);
  }
  size_t added = 0;
  for (size_t i = 0; i < case_count && added < SELF_TEST_MAX_IDS; ++i) {
    const auto &c = CASES[i];
    if (!master_add_id_schedule(c.id, c.interval_ms)) {
      ESP_LOGW(TAG, "Schedule full while adding self-test ID 0x%02X", c.id);
      continue;
    }
    uint8_t payload[LIN_MAX_DATA_SIZE]{};
    for (uint8_t k = 0; k < c.payload_len && k < LIN_MAX_DATA_SIZE; ++k) {
      payload[k] = (uint8_t) (0xA0 + c.id + k);
    }
    set_response_data(c.id, payload, c.payload_len);
    self_test_stats_[added] = {.id = c.id,
                               .scheduled_interval_ms = c.interval_ms,
                               .seen_updates = 0,
                               .last_update_time_us = 0,
                               .first_update_time_us = 0,
                               .missed_windows = 0,
                               .prev_update_time_us = 0,
                               .min_interval_us = 0,
                               .max_interval_us = 0};
    added++;
  }
  self_test_start_time_us_ = esp_timer_get_time();
  self_test_initialized_ = true;
  ESP_LOGI(TAG, "LIN self-test initialized with %zu IDs", added);
}

void LINCommunication::selfTestEvaluate() {
  int64_t elapsed_us = esp_timer_get_time() - self_test_start_time_us_;
  static int64_t last_eval_us = 0;
  if (elapsed_us - last_eval_us < 2'000'000)
    return;
  last_eval_us = elapsed_us;

  ESP_LOGI(TAG, "---- LIN Self-Test Status (%.2fs) ----", elapsed_us / 1e6f);
  bool all_have_updates = true;
  for (size_t i = 0; i < SELF_TEST_MAX_IDS; ++i) {
    const auto &st = self_test_stats_[i];
    if (st.id == 0)
      continue;
    int64_t expected_interval_us = (int64_t) st.scheduled_interval_ms * 1000;
    bool pass = st.seen_updates >= 2;
    all_have_updates &= pass;
    ESP_LOGI(TAG, "ID 0x%02X upd=%u miss=%u min=%.1fms max=%.1fms exp=%.1fms %s", st.id, st.seen_updates,
             st.missed_windows, st.min_interval_us / 1000.0, st.max_interval_us / 1000.0, expected_interval_us / 1000.0,
             pass ? "OK" : "WAIT");
  }
  if (all_have_updates && elapsed_us > 5'000'000) {
    ESP_LOGI(TAG, "LIN self-test: BASIC PASS (all IDs active).");
  }
}

// Statistics methods (low overhead wrappers)
const lin_statistics_t &LINCommunication::getStatistics() const {
  if (lin_protocol_handler_) {
    return lin_protocol_handler_->getStatistics();
  }
  // Return empty statistics if not initialized
  static lin_statistics_t empty_stats = {};
  return empty_stats;
}

void LINCommunication::logStatistics() const {
  if (lin_protocol_handler_) {
    lin_protocol_handler_->logStatistics();
  }
}

void LINCommunication::resetStatistics() {
  if (lin_protocol_handler_) {
    lin_protocol_handler_->resetStatistics();
  }
}

void LINCommunication::updateStatistics() {
  if (lin_protocol_handler_) {
    lin_protocol_handler_->updateStatistics();
  }
}
