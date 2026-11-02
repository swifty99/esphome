#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ONE_WIRE_RMT

#include <cstddef>
#include <cstdint>

#include "esphome/components/one_wire/one_wire_bus.h"
#include "rmt_one_wire_codec.h"

namespace esphome::gpio::rmt_one_wire {

class Port {
 public:
  virtual bool line_is_high() = 0;
  /// Arms RX. False: hardware error.
  virtual bool arm_rx(Symbol *buffer, size_t capacity, uint32_t idle_us) = 0;
  /// Starts TX. count never exceeds max_tx_slots. False: hardware error.
  virtual bool start_tx(const Symbol *symbols, size_t count) = 0;
  /// Stops both channels; no further completion belongs to the aborted section. False: hardware error.
  virtual bool abort() = 0;
  virtual uint32_t now_ms() = 0;
  /// Sleeps until a completion or at most max_ms. Only used by the blocking methods.
  virtual void wait_for_event(uint32_t max_ms) = 0;
  /// Asks the component to keep calling poll(), or to stop calling it.
  virtual void set_loop_active(bool active) = 0;
};

class Engine {
 public:
  /// RX idle outside RESET: max(100, slot + 20) us. The RESET section keeps 600 us.
  Engine(Port *port, size_t mem_words, const Timing &timing = {});

  // Main loop only
  bool submit(one_wire::Transaction *transaction);
  void poll();
  bool is_idle() const;
  void set_failed();  ///< All pending and future work ends in HARDWARE_ERROR

  // ISR safe: store and return
  void on_tx_done();
  void on_rx_done(size_t num_symbols);

  // Blocking facade; each drains the FIFO first, sleeps through Port::wait_for_event
  int reset_blocking();  ///< -1 bus low or failure, 0 no device, 1 present
  /// False on any failure.
  bool write_blocking(const uint8_t *data, size_t num_bits);
  /// On failure, every bit of data is set to 1 and the result is false.
  bool read_blocking(uint8_t *data, size_t num_bits);
  void reset_search();
  uint64_t search_next();  ///< Same contract as OneWireBus::search_int(): 0 when finished or on failure
};

}  // namespace esphome::gpio::rmt_one_wire

#endif  // USE_ONE_WIRE_RMT
