#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ONE_WIRE_RMT

#include <cstddef>
#include <cstdint>

namespace esphome::gpio::rmt_one_wire {

/// Same layout as rmt_symbol_word_t (static_assert in L2), usable without IDF headers.
struct Symbol {
  uint16_t duration0 : 15;
  uint16_t level0 : 1;
  uint16_t duration1 : 15;
  uint16_t level1 : 1;
};

enum class ResetResult : uint8_t { PRESENT, NO_DEVICE, BUS_LOW, CAPTURE_ERROR };

constexpr size_t max_tx_slots(size_t mem_words) { return mem_words - 1; }
constexpr size_t max_rx_slots(size_t mem_words) { return mem_words - 2; }

Symbol reset_symbol();
/// Encodes bits [bit_offset, bit_offset + num_bits) of data, LSB first. Returns num_bits.
size_t encode_write_slots(const uint8_t *data, size_t bit_offset, size_t num_bits, Symbol *out);
size_t encode_read_slots(size_t num_bits, Symbol *out);
ResetResult decode_reset(const Symbol *capture, size_t count);
/// Stores num_bits at bit_offset of out, LSB first. False if the capture is invalid (section 7),
/// out is then undefined. Only the bits being decoded may be changed.
bool decode_read_slots(const Symbol *capture, size_t count, size_t num_bits, uint8_t *out, size_t bit_offset);

}  // namespace esphome::gpio::rmt_one_wire

#endif  // USE_ONE_WIRE_RMT
