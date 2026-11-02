#pragma once

// Hardware independent parts of the RMT 1-wire driver: bus timings, decoding of captured pulses and
// the ROM search. Nothing here touches the RMT peripheral, so it can be unit tested on the host.

#include <cstddef>
#include <cstdint>

namespace esphome::gpio::one_wire_rmt {

// Timings in microseconds for standard speed, from Maxim application note 126.
// The RMT runs at 1 MHz, so one tick is one microsecond.
static constexpr uint16_t RESET_LOW_US = 480;
static constexpr uint16_t RESET_RELEASE_US = 480;
static constexpr uint16_t WRITE1_LOW_US = 6;
static constexpr uint16_t WRITE1_HIGH_US = 64;
static constexpr uint16_t WRITE0_LOW_US = 60;
static constexpr uint16_t WRITE0_HIGH_US = 10;
static constexpr uint16_t READ_LOW_US = 6;
static constexpr uint16_t READ_HIGH_US = 64;

/// A device that sends a 0 holds the bus low for at least 15 µs after the falling edge, a 1 leaves it to the master
/// pulse (READ_LOW_US plus the rise time). Pick the middle, so both sides keep a margin.
static constexpr uint16_t READ_THRESHOLD_US = 11;

/// A presence pulse starts 15 to 60 µs after the release and lasts 60 to 240 µs. Be a little more tolerant than that.
static constexpr uint16_t PRESENCE_WAIT_MAX_US = 120;
static constexpr uint16_t PRESENCE_LOW_MIN_US = 30;
/// A reset pulse longer than this means something else keeps the bus low.
static constexpr uint16_t RESET_STUCK_US = RESET_LOW_US + 60;

/// A stretch of time in which the bus stayed at one level.
struct Phase {
  uint8_t level;
  uint32_t duration;
};

/// Convert captured RMT symbols to alternating phases. Zero length phases (the end marker of a capture) are dropped
/// and neighbours with the same level are merged. Works with any type that has `level0`, `duration0`, `level1` and
/// `duration1`, like `rmt_symbol_word_t`. Returns the number of phases written to `out`.
template<typename Symbol> size_t flatten(const Symbol *symbols, size_t count, Phase *out, size_t max_out) {
  size_t n = 0;
  auto add = [&](uint8_t level, uint32_t duration) {
    if (duration == 0)
      return;
    if (n > 0 && out[n - 1].level == level) {
      out[n - 1].duration += duration;
    } else if (n < max_out) {
      out[n++] = {level, duration};
    }
  };
  for (size_t i = 0; i < count; i++) {
    add(symbols[i].level0, symbols[i].duration0);
    add(symbols[i].level1, symbols[i].duration1);
  }
  return n;
}

/// Turn the low pulses in `phases` into bits (bit 0 is the first pulse): a long pulse is a 0, a short one a 1.
/// Returns how many low pulses were seen. At most 64 are decoded.
inline size_t decode_bits(const Phase *phases, size_t count, uint64_t &bits) {
  size_t pulses = 0;
  bits = 0;
  for (size_t i = 0; i < count; i++) {
    if (phases[i].level != 0)
      continue;
    if (pulses < 64 && phases[i].duration <= READ_THRESHOLD_US)
      bits |= uint64_t(1) << pulses;
    pulses++;
  }
  return pulses < 64 ? pulses : 64;
}

enum class ResetResult : int8_t {
  RESET_RESULT_FAULT = -1,     ///< no usable reset pulse, or the bus is held low
  RESET_RESULT_NO_DEVICE = 0,  ///< the bus came back high and nobody answered
  RESET_RESULT_PRESENT = 1,    ///< at least one device sent a presence pulse
};

inline ResetResult classify_reset(const Phase *phases, size_t count) {
  size_t i = 0;
  while (i < count && phases[i].level != 0)
    i++;
  if (i == count || phases[i].duration > RESET_STUCK_US)
    return ResetResult::RESET_RESULT_FAULT;
  // After the reset pulse the bus is released (high), then a device pulls it low for the presence pulse
  if (i + 2 < count && phases[i + 1].duration <= PRESENCE_WAIT_MAX_US && phases[i + 2].level == 0 &&
      phases[i + 2].duration >= PRESENCE_LOW_MIN_US)
    return ResetResult::RESET_RESULT_PRESENT;
  return ResetResult::RESET_RESULT_NO_DEVICE;
}

/// Split the pulses of a search step into the address bit and its complement. The step is an optional write slot,
/// followed by two read slots. `bits` is the output of decode_bits(), `pulses` its return value.
inline bool split_search_bits(uint64_t bits, size_t pulses, bool write_first, bool &id_bit, bool &cmp_id_bit) {
  size_t expected = write_first ? 3 : 2;
  if (pulses != expected)
    return false;
  if (write_first)
    bits >>= 1;  // the first pulse is the one that was written
  id_bit = (bits & 1) != 0;
  cmp_id_bit = (bits & 2) != 0;
  return true;
}

/// Find the next device on the bus, with the algorithm from Maxim application note 187.
///
/// `Io` provides
///  - `bool search_bits(bool write_first, bool bit, bool &id_bit, bool &cmp_id_bit)`: optionally write the branch
///    chosen for the previous address bit, then read the address bit and its complement.
///  - `bool write_bit(bool bit)`: write the branch chosen for the last address bit.
/// Both return false on a bus failure.
///
/// The three state variables are the ones the caller keeps between calls. Returns 0 if there are no more devices.
template<typename Io>
uint64_t search_next(Io &io, uint8_t &last_discrepancy, bool &last_device_flag, uint64_t &last_address) {
  if (last_device_flag)
    return 0;

  uint8_t last_zero = 0;
  uint64_t bit_mask = 1;
  uint64_t address = last_address;
  bool branch = false;

  for (uint8_t bit_number = 1; bit_number <= 64; bit_number++, bit_mask <<= 1) {
    bool id_bit, cmp_id_bit;
    if (!io.search_bits(bit_number > 1, branch, id_bit, cmp_id_bit))
      return 0;
    if (id_bit && cmp_id_bit)
      return 0;  // no devices are taking part

    if (id_bit != cmp_id_bit) {
      // all remaining devices have the same bit here
      branch = id_bit;
    } else {
      // there are devices with a 0 and with a 1 at this position
      if (bit_number < last_discrepancy) {
        branch = (address & bit_mask) != 0;
      } else {
        branch = bit_number == last_discrepancy;
      }
      if (!branch)
        last_zero = bit_number;
    }

    if (branch) {
      address |= bit_mask;
    } else {
      address &= ~bit_mask;
    }
  }
  if (!io.write_bit(branch))
    return 0;

  last_discrepancy = last_zero;
  if (last_discrepancy == 0)
    last_device_flag = true;  // nothing left to branch into, this was the last device
  last_address = address;
  return address;
}

}  // namespace esphome::gpio::one_wire_rmt
