#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include "esphome/components/gpio/one_wire/one_wire_rmt_decode.h"

namespace esphome::gpio::testing {

using namespace one_wire_rmt;

struct Symbol {
  uint32_t duration0;
  uint32_t level0;
  uint32_t duration1;
  uint32_t level1;
};

inline std::vector<Phase> phases_of(const std::vector<Symbol> &symbols) {
  std::vector<Phase> phases(2 * symbols.size());
  phases.resize(flatten(symbols.data(), symbols.size(), phases.data(), phases.size()));
  return phases;
}

// A bus with devices on it. A device takes part in a search step with the bits of its address, and drops out
// when the master announces another branch. The bus is a wired AND: a 0 from any device wins.
class SimulatedBus {
 public:
  explicit SimulatedBus(std::vector<uint64_t> devices) : devices_(std::move(devices)) { this->start(); }

  void start() {
    this->active_ = this->devices_;
    this->position_ = 0;
  }

  bool search_bits(bool write_first, bool bit, bool &id_bit, bool &cmp_id_bit) {
    if (this->fail_at_step_ == this->steps_++)
      return false;
    if (write_first)
      this->write_bit(bit);
    id_bit = true;
    cmp_id_bit = true;
    for (uint64_t device : this->active_) {
      bool value = (device >> this->position_) & 1;
      id_bit = id_bit && value;
      cmp_id_bit = cmp_id_bit && !value;
    }
    if (this->active_.empty())
      id_bit = cmp_id_bit = true;  // nobody pulls the bus low
    return true;
  }

  bool write_bit(bool bit) {
    std::erase_if(this->active_, [&](uint64_t device) { return ((device >> this->position_) & 1) != bit; });
    this->position_++;
    return true;
  }

  int fail_at_step_{-1};
  int steps_{0};

 protected:
  std::vector<uint64_t> devices_;
  std::vector<uint64_t> active_;
  int position_{0};
};

inline std::vector<uint64_t> enumerate(const std::vector<uint64_t> &devices) {
  SimulatedBus bus(devices);
  uint8_t last_discrepancy = 0;
  bool last_device_flag = false;
  uint64_t address = 0;
  std::vector<uint64_t> found;
  for (size_t i = 0; i < devices.size() + 2; i++) {
    bus.start();
    uint64_t next = search_next(bus, last_discrepancy, last_device_flag, address);
    if (next == 0)
      break;
    found.push_back(next);
  }
  return found;
}

}  // namespace esphome::gpio::testing
