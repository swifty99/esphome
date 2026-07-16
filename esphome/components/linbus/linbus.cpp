#include "linbus.h"
#include "esphome/core/log.h"
#include "esphome/core/gpio.h"
#include "soc/soc_caps.h"

namespace esphome {
namespace linbus {

static const char *const TAG = "linbus";

// Helper to get pin number from GPIOPin
static int get_pin_no(GPIOPin *pin) {
  if (pin == nullptr || !pin->is_internal())
    return -1;
  return ((InternalGPIOPin *) pin)->get_pin();
}

void LinBusComponent::set_uart_port_str(const std::string &port) {
  if (port == "UART_NUM_0")
    this->uart_port_ = UART_NUM_0;
  else if (port == "UART_NUM_1")
    this->uart_port_ = UART_NUM_1;
#if SOC_UART_HP_NUM > 2
  else if (port == "UART_NUM_2")
    this->uart_port_ = UART_NUM_2;
#endif
  else {
    ESP_LOGW(TAG, "Unknown UART port: %s, defaulting to UART_NUM_1", port.c_str());
    this->uart_port_ = UART_NUM_1;
  }
}

void LinBusComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LIN Bus...");

  // Configure pins
  if (this->tx_pin_ != nullptr) {
    this->tx_pin_->setup();
  }
  if (this->rx_pin_ != nullptr) {
    this->rx_pin_->setup();
  }
  if (this->cs_pin_ != nullptr) {
    this->cs_pin_->setup();
    this->cs_pin_->digital_write(true);  // Enable transceiver
  }

  // Create LINCommunication instance
  int tx_pin = get_pin_no(this->tx_pin_);
  int rx_pin = get_pin_no(this->rx_pin_);
  int cs_pin = get_pin_no(this->cs_pin_);

  this->lin_comm_ = new LINCommunication(this->uart_port_, tx_pin, rx_pin, cs_pin);

  // Initialize LIN communication
  esp_err_t err = this->lin_comm_->init(this->mode_, this->baud_rate_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize LIN communication: %d", err);
    this->mark_failed();
    return;
  }

  // Apply schedule configuration
  for (const auto &item : this->pending_schedule_) {
    this->lin_comm_->master_add_id_schedule(item.id, item.interval_ms);
  }
  this->pending_schedule_.clear();

  // Apply response configuration
  for (const auto &resp : this->pending_responses_) {
    this->lin_comm_->set_response_data(resp.id, const_cast<uint8_t *>(resp.data.data()), resp.data.size(),
                                       resp.enhanced);
    // A published response defines this ID's frame length and checksum mode, so register it as
    // an RX hint too — the echo of our own frame then finalizes deterministically (A3/N5).
    this->lin_comm_->configure_rx_hint(resp.id, static_cast<uint8_t>(resp.data.size()), resp.enhanced ? 1 : 0);
  }
  this->pending_responses_.clear();

  // Apply explicit per-received-ID hints (rx_ids, on_frame/schedule length + checksum).
  for (const auto &hint : this->pending_rx_hints_) {
    this->lin_comm_->configure_rx_hint(hint.id, hint.length, hint.checksum_mode);
  }
  this->pending_rx_hints_.clear();

  // Enable the built-in loopback self-test if configured (master + transceiver + pull-up).
  if (this->self_test_) {
    this->lin_comm_->enableSelfTest(true);
  }

  ESP_LOGCONFIG(TAG, "LIN Bus setup complete");
}

void LinBusComponent::loop() {
  if (this->lin_comm_ == nullptr)
    return;

  // Process LIN communication (master scheduling, statistics, etc.)
  this->lin_comm_->lin_process();

  // Fire on_error for each newly-counted fault (loop runs ~16 ms, so a deliberately induced
  // error lights up within a frame or two). Skipped entirely when no on_error is configured.
  if (!this->error_triggers_.empty()) {
    const lin_statistics_t &est = this->lin_comm_->getStatistics();
    const struct {
      uint32_t cur;
      uint32_t *last;
      const char *kind;
    } checks[] = {
        {est.checksum_errors_total, &this->last_checksum_errors_, "checksum"},
        {est.frame_errors_total, &this->last_frame_errors_, "framing"},
        {est.pid_errors_total, &this->last_pid_errors_, "pid"},
        {est.collisions_total, &this->last_collisions_, "collision"},
        {est.master_send_failures_total, &this->last_send_failures_, "send"},
    };
    for (const auto &c : checks) {
      if (c.cur != *c.last) {
        *c.last = c.cur;
        for (auto *trigger : this->error_triggers_)
          trigger->fire(c.kind);
      }
    }
  }

  // Update sensors and log statistics every 10 seconds
  uint32_t now = millis();
  if (now - this->last_stats_log_ > 10000) {
    this->last_stats_log_ = now;
    const lin_statistics_t &stats = this->lin_comm_->getStatistics();

    // Publish sensor values. The counter sensors are STATE_CLASS_TOTAL_INCREASING, so they
    // must be fed the monotonic lifetime totals, not the per-window counters that reset
    // every 10 s (which would look like meter resets to Home Assistant). See UPGRADES.md H4.
    if (this->master_requests_sensor_)
      this->master_requests_sensor_->publish_state(stats.master_requests_total);
    if (this->id_requests_answered_sensor_)
      this->id_requests_answered_sensor_->publish_state(stats.id_requests_answered_total);
    if (this->data_bytes_received_sensor_)
      this->data_bytes_received_sensor_->publish_state(stats.data_bytes_received_total);
    if (this->frames_received_sensor_)
      this->frames_received_sensor_->publish_state(stats.frames_received_total);
    if (this->uart_breaks_sensor_)
      this->uart_breaks_sensor_->publish_state(stats.uart_breaks_total);
    if (this->checksum_errors_sensor_)
      this->checksum_errors_sensor_->publish_state(stats.checksum_errors_total);
    if (this->checksum_fallback_sensor_)
      this->checksum_fallback_sensor_->publish_state(stats.checksum_fallback_total);
    if (this->pid_errors_sensor_)
      this->pid_errors_sensor_->publish_state(stats.pid_errors_total);
    if (this->framing_errors_sensor_)
      this->framing_errors_sensor_->publish_state(stats.frame_errors_total);
    if (this->collisions_sensor_)
      this->collisions_sensor_->publish_state(stats.collisions_total);
    if (this->frames_coalesced_sensor_)
      this->frames_coalesced_sensor_->publish_state(stats.frames_coalesced_total);
    if (this->send_failures_sensor_)
      this->send_failures_sensor_->publish_state(stats.master_send_failures_total);
    if (this->bus_load_sensor_)
      this->bus_load_sensor_->publish_state(stats.estimated_bus_load_percent);
    if (this->error_rate_sensor_)
      this->error_rate_sensor_->publish_state(stats.error_rate_percent);
    if (this->self_test_pass_sensor_)
      this->self_test_pass_sensor_->publish_state(this->lin_comm_->self_test_passed());

    ESP_LOGI(TAG, "=== LIN Bus Statistics ===");
    ESP_LOGI(TAG, "Master Requests: %u", stats.master_requests_sent);
    ESP_LOGI(TAG, "ID Answers: %u", stats.id_requests_answered);
    ESP_LOGI(TAG, "Data Bytes RX: %u", stats.data_bytes_received);
    ESP_LOGI(TAG, "Total Bus Bytes: %u", stats.total_bus_bytes);
    ESP_LOGI(TAG, "Unique IDs Seen: %u", stats.unique_ids_seen);
    ESP_LOGI(TAG, "Checksum Errors: %u", stats.checksum_errors);
    ESP_LOGI(TAG, "Send Failures: %u", stats.master_send_failures);
    ESP_LOGI(TAG, "Bus Load: %.1f%%", stats.estimated_bus_load_percent);
    ESP_LOGI(TAG, "Error Rate: %.2f%%", stats.error_rate_percent);
  }

  // Check for new frames and trigger automations
  for (auto *trigger : this->frame_triggers_) {
    uint8_t lin_id = trigger->get_lin_id();
    lin_discovered_id_t frame_data;

    // Get current frame data for this LIN ID
    if (this->lin_comm_->get_response_4id(lin_id, &frame_data)) {
      // Check if this is a new frame (updateCount increased)
      auto it = this->last_update_counts_.find(lin_id);
      uint32_t last_count = (it != this->last_update_counts_.end()) ? it->second : 0;

      // Fire on any change, not just an increase: after LRU eviction + rediscovery the
      // discovered-table updateCount restarts at 1, which is < a stale high-water mark and
      // would otherwise silence the trigger forever. See UPGRADES.md H2.
      if (frame_data.updateCount != last_count) {
        // Frames that arrived between loop iterations were coalesced into this one update —
        // count them so the loss is visible, not silent (H3/N6). Only when the count advanced
        // monotonically (skip the H2 eviction/rediscovery restart, where it drops).
        if (frame_data.updateCount > last_count && (frame_data.updateCount - last_count) > 1) {
          this->lin_comm_->note_frames_coalesced(frame_data.updateCount - last_count - 1);
        }

        // New frame received! Fire the trigger
        std::vector<uint8_t> data_vec(frame_data.data, frame_data.data + frame_data.dataLength);
        trigger->trigger_frame(data_vec, frame_data.dataLength);

        // Update last seen count
        this->last_update_counts_[lin_id] = frame_data.updateCount;

        ESP_LOGD(TAG, "Fired trigger for LIN ID 0x%02X (count: %u -> %u)", lin_id, last_count, frame_data.updateCount);
      }
    }
  }
}

void LinBusComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "LIN Bus:");
  ESP_LOGCONFIG(TAG, "  UART Port: %d", this->uart_port_);
  ESP_LOGCONFIG(TAG, "  Baud Rate: %u", this->baud_rate_);
  LOG_PIN("  TX Pin: ", this->tx_pin_);
  LOG_PIN("  RX Pin: ", this->rx_pin_);
  LOG_PIN("  CS Pin: ", this->cs_pin_);
  const char *mode_str = this->mode_ == LIN_MODE_MASTER  ? "Master"
                         : this->mode_ == LIN_MODE_SLAVE ? "Slave"
                                                         : "Listener";
  ESP_LOGCONFIG(TAG, "  Mode: %s", mode_str);
  ESP_LOGCONFIG(TAG, "  Self-Test: %s", this->self_test_ ? "enabled" : "disabled");
}

void LinBusComponent::add_schedule_item(uint8_t id, uint32_t interval_ms) {
  // Store for later application in setup()
  this->pending_schedule_.push_back({id, interval_ms});
}

void LinBusComponent::add_response(uint8_t id, const std::vector<uint8_t> &data, bool enhanced) {
  // Store for later application in setup()
  this->pending_responses_.push_back({id, data, enhanced});
}

void LinBusComponent::add_rx_hint(uint8_t id, uint8_t length, int8_t checksum_mode) {
  // Store for later application in setup() (the protocol handler exists only after init()).
  this->pending_rx_hints_.push_back({id, length, checksum_mode});
}

void LinBusComponent::run_self_test(bool enable) {
  if (this->lin_comm_ == nullptr) {
    ESP_LOGW(TAG, "Cannot run self-test: LIN communication not initialized");
    return;
  }
  ESP_LOGI(TAG, "%s LIN self-test", enable ? "Starting" : "Stopping");
  this->lin_comm_->enableSelfTest(enable);
}

void LinBusComponent::update_response(uint8_t lin_id, const std::vector<uint8_t> &data) {
  if (this->lin_comm_ == nullptr) {
    ESP_LOGW(TAG, "Cannot update response: LIN communication not initialized");
    return;
  }

  if (data.size() > 8) {
    ESP_LOGW(TAG, "Cannot update response for ID 0x%02X: data too long (%zu bytes, max 8)", lin_id, data.size());
    return;
  }

  // Update the response data
  bool success =
      this->lin_comm_->set_response_data(lin_id, const_cast<uint8_t *>(data.data()), data.size(), true, true);

  if (success) {
    ESP_LOGD(TAG, "Updated response for LIN ID 0x%02X with %zu bytes", lin_id, data.size());
  } else {
    ESP_LOGW(TAG, "Failed to update response for LIN ID 0x%02X", lin_id);
  }
}

// Statistics getters
uint32_t LinBusComponent::get_master_requests_sent() const {
  if (this->lin_comm_ == nullptr)
    return 0;
  return this->lin_comm_->getStatistics().master_requests_sent;
}

uint32_t LinBusComponent::get_id_requests_answered() const {
  if (this->lin_comm_ == nullptr)
    return 0;
  return this->lin_comm_->getStatistics().id_requests_answered;
}

uint32_t LinBusComponent::get_data_bytes_received() const {
  if (this->lin_comm_ == nullptr)
    return 0;
  return this->lin_comm_->getStatistics().data_bytes_received;
}

uint32_t LinBusComponent::get_checksum_errors() const {
  if (this->lin_comm_ == nullptr)
    return 0;
  return this->lin_comm_->getStatistics().checksum_errors;
}

uint32_t LinBusComponent::get_master_send_failures() const {
  if (this->lin_comm_ == nullptr)
    return 0;
  return this->lin_comm_->getStatistics().master_send_failures;
}

float LinBusComponent::get_bus_load_percent() const {
  if (this->lin_comm_ == nullptr)
    return 0.0f;
  return this->lin_comm_->getStatistics().estimated_bus_load_percent;
}

float LinBusComponent::get_error_rate_percent() const {
  if (this->lin_comm_ == nullptr)
    return 0.0f;
  return this->lin_comm_->getStatistics().error_rate_percent;
}

}  // namespace linbus
}  // namespace esphome
