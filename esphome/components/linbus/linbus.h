#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
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

// Trigger fired once per newly-counted error, with the fault class as a string
// ("checksum" | "framing" | "pid" | "collision" | "send"). Lets a bench LED / log / alert
// react to any fault — including deliberately induced ones — without polling counters.
class LinErrorTrigger : public Trigger<std::string> {
 public:
  void fire(const std::string &kind) { this->trigger(kind); }
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
  // Per-received-ID hint from config (length-aware finalize / declared checksum mode).
  // length 0 = no length hint; checksum_mode -1 = undeclared, 0 = classic, 1 = enhanced.
  void add_rx_hint(uint8_t id, uint8_t length, int8_t checksum_mode);
  void set_self_test(bool enable) { self_test_ = enable; }

  // Register on_frame / on_error triggers
  void register_frame_trigger(LinFrameTrigger *trigger) { frame_triggers_.push_back(trigger); }
  void register_error_trigger(LinErrorTrigger *trigger) { error_triggers_.push_back(trigger); }

  // Actions
  void update_response(uint8_t lin_id, const std::vector<uint8_t> &data);
  void run_self_test(bool enable);

  // Statistics getters
  uint32_t get_master_requests_sent() const;
  uint32_t get_id_requests_answered() const;
  uint32_t get_data_bytes_received() const;
  uint32_t get_checksum_errors() const;
  uint32_t get_master_send_failures() const;
  float get_bus_load_percent() const;
  float get_error_rate_percent() const;

  // Sensor registration
  void set_master_requests_sensor(sensor::Sensor *sensor) { master_requests_sensor_ = sensor; }
  void set_id_requests_answered_sensor(sensor::Sensor *sensor) { id_requests_answered_sensor_ = sensor; }
  void set_data_bytes_received_sensor(sensor::Sensor *sensor) { data_bytes_received_sensor_ = sensor; }
  void set_frames_received_sensor(sensor::Sensor *sensor) { frames_received_sensor_ = sensor; }
  void set_uart_breaks_sensor(sensor::Sensor *sensor) { uart_breaks_sensor_ = sensor; }
  void set_checksum_errors_sensor(sensor::Sensor *sensor) { checksum_errors_sensor_ = sensor; }
  void set_checksum_fallback_sensor(sensor::Sensor *sensor) { checksum_fallback_sensor_ = sensor; }
  void set_pid_errors_sensor(sensor::Sensor *sensor) { pid_errors_sensor_ = sensor; }
  void set_framing_errors_sensor(sensor::Sensor *sensor) { framing_errors_sensor_ = sensor; }
  void set_collisions_sensor(sensor::Sensor *sensor) { collisions_sensor_ = sensor; }
  void set_frames_coalesced_sensor(sensor::Sensor *sensor) { frames_coalesced_sensor_ = sensor; }
  void set_send_failures_sensor(sensor::Sensor *sensor) { send_failures_sensor_ = sensor; }
  void set_bus_load_sensor(sensor::Sensor *sensor) { bus_load_sensor_ = sensor; }
  void set_error_rate_sensor(sensor::Sensor *sensor) { error_rate_sensor_ = sensor; }
  void set_self_test_pass_binary_sensor(binary_sensor::BinarySensor *bs) { self_test_pass_sensor_ = bs; }

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
  struct RxHintItem {
    uint8_t id;
    uint8_t length;        // 0 = no length hint
    int8_t checksum_mode;  // -1 undeclared, 0 classic, 1 enhanced
  };
  std::vector<ScheduleItem> pending_schedule_;
  std::vector<ResponseItem> pending_responses_;
  std::vector<RxHintItem> pending_rx_hints_;
  bool self_test_{false};

  // Frame / error triggers for automation
  std::vector<LinFrameTrigger *> frame_triggers_;
  std::vector<LinErrorTrigger *> error_triggers_;

  // Track last seen update count for each LIN ID to detect new frames
  std::unordered_map<uint8_t, uint32_t> last_update_counts_;

  // Last-seen error lifetime totals, so loop() can fire on_error on each new fault.
  uint32_t last_checksum_errors_{0};
  uint32_t last_frame_errors_{0};
  uint32_t last_pid_errors_{0};
  uint32_t last_collisions_{0};
  uint32_t last_send_failures_{0};

  // Per-instance stats-log throttle (was a function-local static, UPGRADES.md M6)
  uint32_t last_stats_log_{0};

  // Sensors for statistics
  sensor::Sensor *master_requests_sensor_{nullptr};
  sensor::Sensor *id_requests_answered_sensor_{nullptr};
  sensor::Sensor *data_bytes_received_sensor_{nullptr};
  sensor::Sensor *frames_received_sensor_{nullptr};
  sensor::Sensor *uart_breaks_sensor_{nullptr};
  sensor::Sensor *checksum_errors_sensor_{nullptr};
  sensor::Sensor *checksum_fallback_sensor_{nullptr};
  sensor::Sensor *pid_errors_sensor_{nullptr};
  sensor::Sensor *framing_errors_sensor_{nullptr};
  sensor::Sensor *collisions_sensor_{nullptr};
  sensor::Sensor *frames_coalesced_sensor_{nullptr};
  sensor::Sensor *send_failures_sensor_{nullptr};
  sensor::Sensor *bus_load_sensor_{nullptr};
  sensor::Sensor *error_rate_sensor_{nullptr};
  binary_sensor::BinarySensor *self_test_pass_sensor_{nullptr};
};

// Action to update LIN response data
template<typename... Ts> class UpdateResponseAction : public Action<Ts...> {
 public:
  UpdateResponseAction(LinBusComponent *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint8_t, lin_id)

  void set_data_template(std::function<std::vector<uint8_t>(Ts...)> func) { this->data_func_ = func; }
  void set_data_static(const std::vector<uint8_t> &data) { this->data_static_ = data; }

  void play(const Ts &...x) override {
    auto lin_id = this->lin_id_.value(x...);
    std::vector<uint8_t> data;

    if (this->data_func_.has_value()) {
      data = this->data_func_.value()(x...);
    } else {
      data = this->data_static_;
    }

    this->parent_->update_response(lin_id, data);
  }

 protected:
  LinBusComponent *parent_;
  optional<std::function<std::vector<uint8_t>(Ts...)>> data_func_;
  std::vector<uint8_t> data_static_;
};

// Action to start/stop the built-in single-node loopback self-test (S6).
template<typename... Ts> class RunSelfTestAction : public Action<Ts...> {
 public:
  RunSelfTestAction(LinBusComponent *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(bool, enable)

  void play(const Ts &...x) override { this->parent_->run_self_test(this->enable_.value(x...)); }

 protected:
  LinBusComponent *parent_;
};

}  // namespace linbus
}  // namespace esphome
