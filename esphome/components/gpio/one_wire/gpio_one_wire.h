#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/one_wire/one_wire.h"

#ifdef USE_ONE_WIRE_RMT
#include "one_wire_rmt_decode.h"
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#endif

namespace esphome::gpio {

class GPIOOneWireBus final : public one_wire::OneWireBus, public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  void set_pin(InternalGPIOPin *pin) {
    this->t_pin_ = pin;
#ifndef USE_ONE_WIRE_RMT
    this->pin_ = pin->to_isr();
#endif
  }

  void write8(uint8_t val) override;
  void write64(uint64_t val) override;
  uint8_t read8() override;
  uint64_t read64() override;

 protected:
  InternalGPIOPin *t_pin_{};
  uint8_t last_discrepancy_{0};
  bool last_device_flag_{false};
  uint64_t address_{0};

  int reset_int() override;
  void reset_search() override;
  uint64_t search_int() override;

#ifdef USE_ONE_WIRE_RMT
  /// Send `size` bytes of `data` through `encoder`. Returns false on failure.
  bool send_(rmt_encoder_handle_t encoder, const void *data, size_t size, uint32_t duration_us);
  /// Like send_(), and also capture the bus. The levels seen are stored in `phases`.
  bool exchange_(rmt_encoder_handle_t encoder, const void *data, size_t size, uint32_t idle_us, uint32_t duration_us,
                 one_wire_rmt::Phase *phases, size_t &phase_count);
  bool write_bit_(bool bit);
  void report_(bool ok, const LogString *reason);
  void cancel_();
  void destroy_();

  rmt_channel_handle_t tx_channel_{nullptr};
  rmt_channel_handle_t rx_channel_{nullptr};
  rmt_encoder_handle_t bytes_encoder_{nullptr};
  rmt_encoder_handle_t copy_encoder_{nullptr};
  rmt_symbol_word_t *rx_symbols_{nullptr};
  QueueHandle_t rx_queue_{nullptr};
  bool transfer_failed_{false};
#else
  ISRInternalGPIOPin pin_;
  void write_bit_(bool bit);
  bool read_bit_();
  bool read_bit_(uint32_t *t);
#endif
};

}  // namespace esphome::gpio
