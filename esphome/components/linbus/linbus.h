#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/automation.h"
#include "LINCommunication.h"
#include <vector>
#include <string>
#include <unordered_map>

namespace esphome {
namespace linbus {

// Forward declaration
class LinBusComponent;

// Trigger for when a LIN frame is received
class LinFrameTrigger : public Trigger<std::vector<uint8_t>, uint8_t> {
 public:
  LinFrameTrigger(LinBusComponent *parent, uint8_t lin_id) : parent_(parent), lin_id_(lin_id) {}

  uint8_t get_lin_id() const { return lin_id_; }
  void trigger_frame(const std::vector<uint8_t> &data, uint8_t data_length) { this->trigger(data, data_length); }

 protected:
  LinBusComponent *parent_;
  uint8_t lin_id_;
};

class LinBusComponent : public Component {
 public:
  LinBusComponent() = default;

  // ESPHome lifecycle
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  // Configuration setters (called from Python)
  void set_tx_pin(GPIOPin *pin) { tx_pin_ = pin; }
  void set_rx_pin(GPIOPin *pin) { rx_pin_ = pin; }
  void set_cs_pin(GPIOPin *pin) { cs_pin_ = pin; }
  void set_uart_port_str(const std::string &port);
  void set_baud_rate(uint32_t baud) { baud_rate_ = baud; }
  void set_mode(uint8_t mode) { mode_ = static_cast<lin_mode_t>(mode); }

  void add_schedule_item(uint8_t id, uint32_t interval_ms);
  void add_response(uint8_t id, const std::vector<uint8_t> &data, bool enhanced);

  // Register on_frame trigger
  void register_frame_trigger(LinFrameTrigger *trigger) { frame_triggers_.push_back(trigger); }

  // Statistics getters
  uint32_t get_master_requests_sent() const;
  uint32_t get_id_requests_answered() const;
  uint32_t get_data_bytes_received() const;
  uint32_t get_checksum_errors() const;
  uint32_t get_master_send_failures() const;
  float get_bus_load_percent() const;
  float get_error_rate_percent() const;

 protected:
  GPIOPin *tx_pin_{nullptr};
  GPIOPin *rx_pin_{nullptr};
  GPIOPin *cs_pin_{nullptr};
  uart_port_t uart_port_{UART_NUM_1};
  uint32_t baud_rate_{19200};
  lin_mode_t mode_{LIN_MODE_MASTER};

  LINCommunication *lin_comm_{nullptr};

  // Temporary storage for configuration (applied in setup())
  struct ScheduleItem {
    uint8_t id;
    uint32_t interval_ms;
  };
  struct ResponseItem {
    uint8_t id;
    std::vector<uint8_t> data;
    bool enhanced;
  };
  std::vector<ScheduleItem> pending_schedule_;
  std::vector<ResponseItem> pending_responses_;

  // Frame triggers for automation
  std::vector<LinFrameTrigger *> frame_triggers_;

  // Track last seen update count for each LIN ID to detect new frames
  std::unordered_map<uint8_t, uint32_t> last_update_counts_;
};

}  // namespace linbus
}  // namespace esphome
