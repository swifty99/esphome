#include "linbus.h"
#include "esphome/core/log.h"
#include "esphome/core/gpio.h"

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
#if SOC_UART_NUM > 2
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
    this->lin_comm_->set_response_data(resp.id, const_cast<uint8_t *>(resp.data.data()), resp.data.size());
  }
  this->pending_responses_.clear();

  ESP_LOGCONFIG(TAG, "LIN Bus setup complete");
}

void LinBusComponent::loop() {
  if (this->lin_comm_ == nullptr)
    return;

  // Process LIN communication (master scheduling, statistics, etc.)
  this->lin_comm_->lin_process();

  // Log statistics every 10 seconds
  static uint32_t last_stats_log = 0;
  uint32_t now = millis();
  if (now - last_stats_log > 10000) {
    last_stats_log = now;
    const lin_statistics_t &stats = this->lin_comm_->getStatistics();

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

      if (frame_data.updateCount > last_count) {
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
}

void LinBusComponent::add_schedule_item(uint8_t id, uint32_t interval_ms) {
  // Store for later application in setup()
  this->pending_schedule_.push_back({id, interval_ms});
}

void LinBusComponent::add_response(uint8_t id, const std::vector<uint8_t> &data, bool enhanced) {
  // Store for later application in setup()
  this->pending_responses_.push_back({id, data, enhanced});
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
