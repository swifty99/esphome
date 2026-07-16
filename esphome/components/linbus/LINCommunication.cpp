#include "LINCommunication.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

// Project Headers
#include "LinSerial.h"
#include "LinProtocolHandler.h"

static const char *TAG = "LIN_COMM";

LINCommunication::LINCommunication(uart_port_t uart_num, int tx_pin, int rx_pin, int cs_pin)
    : uart_num_(uart_num), tx_pin_(tx_pin), rx_pin_(rx_pin), cs_pin_(cs_pin), mode_(LIN_MODE_LISTENER), baud_rate_(0) {
  // Initialize LinSerial (can be nullptr until init)
  lin_serial_ = nullptr;

  // Initialize arrays to zero
  memset(schedule_, 0, sizeof(schedule_));
  memset(responses_, 0, sizeof(responses_));
  memset(discoveredIDs_, 0, sizeof(discoveredIDs_));
  data_mutex_ = xSemaphoreCreateMutex();
}

LINCommunication::~LINCommunication() {
  // Stop the UART event task BEFORE deleting the objects it dereferences, to avoid a
  // use-after-free on the serial/handler pointers. See UPGRADES.md H8.
  if (uart_event_task_handle_ != nullptr) {
    vTaskDelete(uart_event_task_handle_);
    uart_event_task_handle_ = nullptr;
  }
  delete lin_protocol_handler_;
  lin_protocol_handler_ = nullptr;
  delete lin_serial_;
  lin_serial_ = nullptr;
  if (data_mutex_ != nullptr) {
    vSemaphoreDelete(data_mutex_);
    data_mutex_ = nullptr;
  }
}

esp_err_t LINCommunication::init(lin_mode_t mode, int baud_rate) {
  if (is_initialized_) {
    ESP_LOGW(TAG, "LINCommunication is already initialized. Skipping reinitialization.");
    return ESP_ERR_INVALID_STATE;
  }

  if (baud_rate <= 0) {
    ESP_LOGE(TAG, "Invalid baud rate %d", baud_rate);
    return ESP_ERR_INVALID_ARG;
  }

  mode_ = mode;
  baud_rate_ = baud_rate;

  // Compute the inter-byte timeout BEFORE the event task can run, so the task never
  // latches a 0 -> forced-1-tick queue timeout that chops frames. See UPGRADES.md C1.
  // Approx 3 byte times (10 bits/byte). 3 * 10 * 1e6 / baud
  inter_byte_timeout_us_ = (uint32_t) ((3ULL * 10ULL * 1000000ULL) / (uint32_t) baud_rate_);

  // Initialize LinSerial. A driver-install failure returns an error to mark_failed()
  // instead of aborting into a bootloop. See UPGRADES.md H5.
  lin_serial_ = new LinSerial(uart_num_, tx_pin_, rx_pin_, baud_rate_);
  esp_err_t serr = lin_serial_->init();
  if (serr != ESP_OK) {
    ESP_LOGE(TAG, "LinSerial init failed: %s", esp_err_to_name(serr));
    delete lin_serial_;
    lin_serial_ = nullptr;
    return serr;
  }

  // Now create LinProtocolHandler with valid pointers
  lin_protocol_handler_ =
      new LinProtocolHandler(lin_serial_, baud_rate_, schedule_, discoveredIDs_, responses_, mode_, data_mutex_);

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

  // LIN at <=20 kbit/s needs low latency, not maximum priority. Running above esp_timer /
  // Wi-Fi / BT (configMAX_PRIORITIES - 1) can starve the system. Use a high-but-bounded
  // priority instead. See UPGRADES.md H9.
  UBaseType_t task_prio = (configMAX_PRIORITIES > 19) ? 18 : (UBaseType_t) (configMAX_PRIORITIES - 2);
  BaseType_t ok = xTaskCreatePinnedToCore(uart_event_task_trampoline, "lin_uart_evt", 4096, this, task_prio,
                                          &uart_event_task_handle_, tskNO_AFFINITY);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "Failed to create UART event task");
    return ESP_FAIL;
  }

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

  // Convert desired microsecond inter-byte timeout to FreeRTOS ticks. An N-tick
  // xQueueReceive guarantees only N-1 *full* tick periods (it can expire at the very next
  // tick interrupt), so floor at 2 ticks. See UPGRADES.md C1.
  TickType_t wait_ticks = pdMS_TO_TICKS((self->inter_byte_timeout_us_ + 999) / 1000);  // round up to ms
  if (wait_ticks < 2)
    wait_ticks = 2;

  // Feed every byte already sitting in the RX ring buffer through the decoder. The UART
  // driver posts BREAK / error events when the condition is detected on the WIRE, while the
  // bytes that precede it may still sit unread in the ring (evt.size batching, boot
  // backlog). Resyncing the decoder on such an event before draining those bytes latches a
  // permanent skew between the event stream and the byte stream — measured on the bench as
  // a listener that reboots onto a busy bus and then decodes ~nothing while uart_breaks
  // keeps climbing (HW-6b). Draining first keeps the decoder aligned; the in-band 0x00
  // break byte then does the actual framing (A2).
  auto drain_rx = [self, &buf]() {
    for (;;) {
      int r = uart_read_bytes(self->uart_num_, buf, sizeof(buf), 0);
      if (r <= 0)
        break;
      for (int i = 0; i < r; ++i)
        self->lin_protocol_handler_->receiveDecode(buf[i], 0);
    }
  };

  // Start from a clean slate: bytes that arrived between driver install and this task
  // starting are a mid-stream fragment at best — discard them so the first event the task
  // acts on refers to data it has actually seen (same HW-6b boot-alignment guard).
  uart_flush_input(self->uart_num_);
  xQueueReset(q);

  for (;;) {
    // Wait for UART event OR timeout (idle gap)
    if (xQueueReceive(q, &evt, wait_ticks) == pdTRUE) {
      switch (evt.type) {
        case UART_DATA: {
          size_t remaining = evt.size;
          while (remaining > 0) {
            size_t to_read = remaining > sizeof(buf) ? sizeof(buf) : remaining;
            int r = uart_read_bytes(self->uart_num_, buf, to_read, 0);
            if (r <= 0)
              break;
            remaining -= (size_t) r;
            for (int i = 0; i < r; ++i) {
              self->lin_protocol_handler_->receiveDecode(buf[i], 0);
            }
          }
          // (No manual finalize here; rely on timeout path)
          break;
        }
        case UART_BREAK:
          // Resync on a hardware BREAK — but only after draining the bytes that preceded it
          // (HW-6b): the event refers to the wire NOW, the decoder to the ring buffer THEN.
          drain_rx();
          self->lin_protocol_handler_->onUartBreak();
          break;
        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
          // Corrupt byte on the wire: drop the in-flight frame and count it, rather than
          // parsing garbage into header states (ghost-header risk). See UPGRADES.md C3.
          // Drain first for the same stream-alignment reason as UART_BREAK (HW-6b).
          drain_rx();
          self->lin_protocol_handler_->onFrameError();
          break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
          uart_flush_input(self->uart_num_);
          self->lin_protocol_handler_->onFrameError();
          break;
        default:
          break;
      }
    } else {
      // Queue wait timed out: inter-byte gap -> finalize frame if in DATA
      self->lin_protocol_handler_->onInterByteTimeout();
    }

    // Off-loop schedule tick (A5/N8): the master header cadence lives HERE, not in the main
    // loop, so the bus keeps being polled through OTA writes and heavy-lambda stalls (N2 for
    // the master side). This task wakes on every UART event and at least every wait_ticks
    // (~2 ms), which bounds schedule jitter to ~2 ms + any in-flight frame time. Header TX
    // and response TX (onID) now share this one task context, so they can never interleave
    // on the wire; data_mutex_ + the isBusIdle() gate inside master_send() still guard
    // against main-loop table mutation and mid-frame transmission.
    if (self->mode_ == LIN_MODE_MASTER) {
      self->lin_protocol_handler_->master_send();
    }
  }
}

bool LINCommunication::lin_process() {
  if (lin_protocol_handler_ == nullptr)
    return false;

  // Note: the master schedule is NOT ticked here. Header cadence runs in the UART event
  // task (A5 off-loop tick, see uart_event_task_trampoline), so a blocked main loop never
  // stalls the bus. This call only carries the loop-context housekeeping below.

  // Built-in self test progression. The self-test's scheduled IDs transmit under the
  // off-loop engine automatically (selfTestInitIfNeeded() adds them via the mutex-protected
  // schedule API); this tick is observation/evaluation only, so a loop stall can delay the
  // verdict but never the exercised traffic.
  if (self_test_enabled_) {
    selfTestTick();
  }

  updateStatistics();
  return true;
}

// master scheduler functions

bool LINCommunication::master_add_id_schedule(uint8_t id, uint32_t interval_ms) {
  if (data_mutex_ != nullptr && xSemaphoreTake(data_mutex_, portMAX_DELAY) != pdTRUE)
    return false;
  // Update existing
  for (size_t i = 0; i < LIN_MAX_ID_RQ_COUNT; i++) {
    if (schedule_[i].updateInterval == 0)
      continue;  // skip free
    if (schedule_[i].id == id) {
      schedule_[i].updateInterval = interval_ms;
      schedule_[i].last_send_time_ms = 0;
      schedule_[i].sent_once = false;
      ESP_LOGI(TAG, "Updated schedule ID 0x%02X interval %u ms", id, interval_ms);
      if (data_mutex_ != nullptr)
        xSemaphoreGive(data_mutex_);
      return true;
    }
  }
  // Find free slot
  for (size_t i = 0; i < LIN_MAX_ID_RQ_COUNT; i++) {
    if (schedule_[i].updateInterval == 0) {
      schedule_[i].id = id;
      schedule_[i].updateInterval = interval_ms;
      schedule_[i].last_send_time_ms = 0;
      schedule_[i].sent_once = false;
      ESP_LOGI(TAG, "Added schedule ID 0x%02X interval %u ms", id, interval_ms);
      if (data_mutex_ != nullptr)
        xSemaphoreGive(data_mutex_);
      return true;
    }
  }
  ESP_LOGW(TAG, "Schedule full, cannot add ID 0x%02X", id);
  if (data_mutex_ != nullptr)
    xSemaphoreGive(data_mutex_);
  return false;
}

bool LINCommunication::master_remove_id_schedule(uint8_t id) {
  if (data_mutex_ != nullptr && xSemaphoreTake(data_mutex_, portMAX_DELAY) != pdTRUE)
    return false;
  for (size_t i = 0; i < LIN_MAX_ID_RQ_COUNT; i++) {
    if (schedule_[i].updateInterval == 0)
      continue;
    if (schedule_[i].id == id) {
      schedule_[i].updateInterval = 0;  // mark free
      schedule_[i].last_send_time_ms = 0;
      schedule_[i].sent_once = false;
      ESP_LOGI(TAG, "Removed schedule ID 0x%02X", id);
      if (data_mutex_ != nullptr)
        xSemaphoreGive(data_mutex_);
      return true;
    }
  }
  ESP_LOGW(TAG, "ID 0x%02X not found in schedule", id);
  if (data_mutex_ != nullptr)
    xSemaphoreGive(data_mutex_);
  return false;
}

// response handling functions
bool LINCommunication::set_response_data(uint8_t id, uint8_t *data, uint8_t data_length, bool enhanced,
                                         bool preserve_existing_checksum) {
  if (id > 0x3F)
    return false;
  if (data_length > LIN_MAX_DATA_SIZE)
    data_length = LIN_MAX_DATA_SIZE;
  if (data_mutex_ != nullptr && xSemaphoreTake(data_mutex_, portMAX_DELAY) != pdTRUE)
    return false;

  // If caller wants to clear: data_length == 0
  if (data_length == 0) {
    for (size_t i = 0; i < LIN_MAX_RESPONSES; ++i) {
      if (responses_[i].dataLength == 0)
        continue;
      if (responses_[i].id == id) {
        responses_[i].dataLength = 0;  // mark free
        // (optional) zero buffer
        memset(responses_[i].responseData, 0, sizeof(responses_[i].responseData));
        responses_[i].enhancedChecksum = true;
        if (data_mutex_ != nullptr)
          xSemaphoreGive(data_mutex_);
        return true;
      }
    }
    if (data_mutex_ != nullptr)
      xSemaphoreGive(data_mutex_);
    return true;  // nothing to clear
  }

  // Update existing
  for (size_t i = 0; i < LIN_MAX_RESPONSES; ++i) {
    if (responses_[i].dataLength == 0)
      continue;
    if (responses_[i].id == id) {
      bool checksum_enhanced = preserve_existing_checksum ? responses_[i].enhancedChecksum : enhanced;
      memcpy(responses_[i].responseData, data, data_length);
      uint8_t cs = lin_protocol_handler_->calculateChecksum(id, data, data_length, checksum_enhanced);
      responses_[i].responseData[data_length] = cs;
      responses_[i].enhancedChecksum = checksum_enhanced;
      responses_[i].dataLength = data_length + 1;
      if (data_mutex_ != nullptr)
        xSemaphoreGive(data_mutex_);
      return true;
    }
  }
  // Find free (dataLength == 0)
  for (size_t i = 0; i < LIN_MAX_RESPONSES; ++i) {
    if (responses_[i].dataLength != 0)
      continue;
    responses_[i].id = id;  // keep ID
    memcpy(responses_[i].responseData, data, data_length);
    uint8_t cs = lin_protocol_handler_->calculateChecksum(id, data, data_length, enhanced);
    responses_[i].responseData[data_length] = cs;
    responses_[i].enhancedChecksum = enhanced;
    responses_[i].dataLength = data_length + 1;
    responses_[i].lastResponseTime = 0;
    if (data_mutex_ != nullptr)
      xSemaphoreGive(data_mutex_);
    return true;
  }
  if (data_mutex_ != nullptr)
    xSemaphoreGive(data_mutex_);
  return false;  // no free slot
}

void LINCommunication::configure_rx_hint(uint8_t id, uint8_t length, int8_t checksum_mode) {
  if (lin_protocol_handler_ == nullptr)
    return;
  if (length > 0)
    lin_protocol_handler_->setRxExpectedLength(id, length);
  if (checksum_mode >= 0)
    lin_protocol_handler_->setRxChecksumMode(id, checksum_mode != 0);
}

void LINCommunication::note_frames_coalesced(uint32_t n) {
  if (lin_protocol_handler_ != nullptr)
    lin_protocol_handler_->noteCoalesced(n);
}

bool LINCommunication::get_response_4id(uint8_t id, lin_discovered_id_t *lindata) {
  if (!lindata) {
    return false;
  }
  if (data_mutex_ != nullptr && xSemaphoreTake(data_mutex_, portMAX_DELAY) != pdTRUE)
    return false;

  // Search for the ID in discoveredIDs_ array
  for (size_t i = 0; i < LIN_MAX_DISCOVERED_PIDS; i++) {
    if (discoveredIDs_[i].dataLength > 0 && discoveredIDs_[i].id == id) {
      // Found the ID, copy data
      *lindata = discoveredIDs_[i];
      if (data_mutex_ != nullptr)
        xSemaphoreGive(data_mutex_);
      return true;
    }
  }

  if (data_mutex_ != nullptr)
    xSemaphoreGive(data_mutex_);
  return false;
}

void LINCommunication::dumpReceivedPIDsToLog() const {
  if (data_mutex_ != nullptr && xSemaphoreTake(data_mutex_, portMAX_DELAY) != pdTRUE)
    return;
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
  if (data_mutex_ != nullptr)
    xSemaphoreGive(data_mutex_);
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
  if (data_mutex_ != nullptr && xSemaphoreTake(data_mutex_, portMAX_DELAY) != pdTRUE)
    return;
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
  if (data_mutex_ != nullptr)
    xSemaphoreGive(data_mutex_);
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
  if (elapsed_us - self_test_last_eval_us_ < 2'000'000)
    return;
  self_test_last_eval_us_ = elapsed_us;

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
    self_test_passed_ = true;
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
