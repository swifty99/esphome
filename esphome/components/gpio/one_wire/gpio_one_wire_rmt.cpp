#include "gpio_one_wire.h"

#ifdef USE_ONE_WIRE_RMT

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <driver/gpio.h>

namespace esphome::gpio {

ESPHOME_LOG_TAG(TAG, "gpio.one_wire");

using one_wire_rmt::Phase;

static constexpr uint32_t RESOLUTION_HZ = 1000000;  // 1 tick = 1 µs
static constexpr uint32_t GLITCH_FILTER_NS = 1000;
// The longest capture is one byte (8 time slots); a search step needs 3.
static constexpr size_t RX_SYMBOLS = 16;
static constexpr size_t MAX_PHASES = 2 * RX_SYMBOLS;
// The bus is idle for this long after the last time slot, which ends a capture.
static constexpr uint32_t SLOT_IDLE_US = 100;
static constexpr uint32_t RESET_IDLE_US = one_wire_rmt::RESET_LOW_US + 100;
static constexpr uint32_t SLOT_US = 70;
static constexpr uint32_t TIMEOUT_MARGIN_MS = 10;

static const rmt_transmit_config_t TX_CONFIG = {
    .loop_count = 0,
    .flags = {.eot_level = 1},  // release the bus when the transmission ends
};

static rmt_symbol_word_t make_symbol(uint16_t low_us, uint16_t high_us) {
  rmt_symbol_word_t symbol = {};
  symbol.level0 = 0;
  symbol.duration0 = low_us;
  symbol.level1 = 1;
  symbol.duration1 = high_us;
  return symbol;
}

static rmt_symbol_word_t write_slot(bool bit) {
  return bit ? make_symbol(one_wire_rmt::WRITE1_LOW_US, one_wire_rmt::WRITE1_HIGH_US)
             : make_symbol(one_wire_rmt::WRITE0_LOW_US, one_wire_rmt::WRITE0_HIGH_US);
}

// Reading is writing a 1, the device pulls the bus low for the time of a 0.
static rmt_symbol_word_t read_slot() { return make_symbol(one_wire_rmt::READ_LOW_US, one_wire_rmt::READ_HIGH_US); }

static bool IRAM_ATTR rx_done(rmt_channel_handle_t, const rmt_rx_done_event_data_t *event, void *queue) {
  BaseType_t woken = pdFALSE;
  xQueueSendFromISR(static_cast<QueueHandle_t>(queue), event, &woken);
  return woken == pdTRUE;
}

void GPIOOneWireBus::destroy_() {
  if (this->bytes_encoder_ != nullptr) {
    rmt_del_encoder(this->bytes_encoder_);
    this->bytes_encoder_ = nullptr;
  }
  if (this->copy_encoder_ != nullptr) {
    rmt_del_encoder(this->copy_encoder_);
    this->copy_encoder_ = nullptr;
  }
  if (this->rx_channel_ != nullptr) {
    rmt_disable(this->rx_channel_);
    rmt_del_channel(this->rx_channel_);
    this->rx_channel_ = nullptr;
  }
  if (this->tx_channel_ != nullptr) {
    rmt_disable(this->tx_channel_);
    rmt_del_channel(this->tx_channel_);
    this->tx_channel_ = nullptr;
  }
  if (this->rx_queue_ != nullptr) {
    vQueueDelete(this->rx_queue_);
    this->rx_queue_ = nullptr;
  }
  if (this->rx_symbols_ != nullptr) {
    RAMAllocator<rmt_symbol_word_t>().deallocate(this->rx_symbols_, RX_SYMBOLS);
    this->rx_symbols_ = nullptr;
  }
}

void GPIOOneWireBus::setup() {
  rmt_bytes_encoder_config_t bytes_config = {};
  bytes_config.bit0 = write_slot(false);
  bytes_config.bit1 = write_slot(true);
  if (rmt_new_bytes_encoder(&bytes_config, &this->bytes_encoder_) != ESP_OK) {
    this->mark_failed(LOG_STR("Creating the bytes encoder failed"));
    return;
  }
  rmt_copy_encoder_config_t copy_config = {};
  if (rmt_new_copy_encoder(&copy_config, &this->copy_encoder_) != ESP_OK) {
    this->destroy_();
    this->mark_failed(LOG_STR("Creating the copy encoder failed"));
    return;
  }

  this->rx_queue_ = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
  RAMAllocator<rmt_symbol_word_t> allocator(RAMAllocator<rmt_symbol_word_t>::ALLOC_INTERNAL);
  this->rx_symbols_ = allocator.allocate(RX_SYMBOLS);
  if (this->rx_queue_ == nullptr || this->rx_symbols_ == nullptr) {
    this->destroy_();
    this->mark_failed(LOG_STR("Out of memory"));
    return;
  }

  auto pin = static_cast<gpio_num_t>(this->t_pin_->get_pin());

  rmt_rx_channel_config_t rx_config = {};
  rx_config.clk_src = RMT_CLK_SRC_DEFAULT;
  rx_config.resolution_hz = RESOLUTION_HZ;
  rx_config.gpio_num = pin;
  rx_config.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
  rmt_rx_event_callbacks_t callbacks = {.on_recv_done = rx_done};
  if (rmt_new_rx_channel(&rx_config, &this->rx_channel_) != ESP_OK ||
      rmt_rx_register_event_callbacks(this->rx_channel_, &callbacks, this->rx_queue_) != ESP_OK) {
    this->destroy_();
    this->mark_failed(LOG_STR("Creating the RMT receive channel failed"));
    return;
  }

  rmt_tx_channel_config_t tx_config = {};
  tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_config.resolution_hz = RESOLUTION_HZ;
  tx_config.gpio_num = pin;
  tx_config.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
  tx_config.trans_queue_depth = 2;
  tx_config.flags.io_loop_back = true;  // the receive channel sees what the transmit channel and the devices do
  tx_config.flags.io_od_mode = true;
  if (rmt_new_tx_channel(&tx_config, &this->tx_channel_) != ESP_OK) {
    this->destroy_();
    this->mark_failed(LOG_STR("Creating the RMT transmit channel failed"));
    return;
  }
  gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);

  if (rmt_enable(this->rx_channel_) != ESP_OK || rmt_enable(this->tx_channel_) != ESP_OK) {
    this->destroy_();
    this->mark_failed(LOG_STR("Enabling the RMT channels failed"));
    return;
  }

  // Put the bus in the released state before the first reset
  rmt_symbol_word_t release = {};
  release.level0 = 1;
  release.duration0 = 1;
  release.level1 = 1;
  if (!this->send_(this->copy_encoder_, &release, sizeof(release), 1)) {
    this->destroy_();
    this->mark_failed(LOG_STR("Releasing the bus failed"));
    return;
  }
  this->search();
}

void GPIOOneWireBus::report_(bool ok, const LogString *reason) {
  if (ok) {
    this->status_clear_warning();
  } else {
    this->status_set_warning(reason);
  }
}

bool GPIOOneWireBus::send_(rmt_encoder_handle_t encoder, const void *data, size_t size, uint32_t duration_us) {
  uint32_t timeout_ms = duration_us / 1000 + TIMEOUT_MARGIN_MS;
  bool ok = rmt_transmit(this->tx_channel_, encoder, data, size, &TX_CONFIG) == ESP_OK &&
            rmt_tx_wait_all_done(this->tx_channel_, timeout_ms) == ESP_OK;
  this->report_(ok, LOG_STR("RMT transmit failed"));
  return ok;
}

bool GPIOOneWireBus::exchange_(rmt_encoder_handle_t encoder, const void *data, size_t size, uint32_t idle_us,
                               uint32_t duration_us, Phase *phases, size_t &phase_count) {
  uint32_t timeout_ms = duration_us / 1000 + TIMEOUT_MARGIN_MS;
  rmt_receive_config_t rx_config = {};
  rx_config.signal_range_min_ns = GLITCH_FILTER_NS;
  rx_config.signal_range_max_ns = idle_us * 1000;

  xQueueReset(this->rx_queue_);
  rmt_rx_done_event_data_t event;
  bool ok =
      rmt_receive(this->rx_channel_, this->rx_symbols_, RX_SYMBOLS * sizeof(rmt_symbol_word_t), &rx_config) == ESP_OK &&
      rmt_transmit(this->tx_channel_, encoder, data, size, &TX_CONFIG) == ESP_OK &&
      xQueueReceive(this->rx_queue_, &event, pdMS_TO_TICKS(timeout_ms)) == pdPASS &&
      rmt_tx_wait_all_done(this->tx_channel_, timeout_ms) == ESP_OK;
  if (!ok) {
    // A capture that never finished would block the next one. Disabling the channel cancels it.
    rmt_disable(this->rx_channel_);
    rmt_enable(this->rx_channel_);
    this->report_(false, LOG_STR("RMT capture failed"));
    return false;
  }
  phase_count = one_wire_rmt::flatten(event.received_symbols, event.num_symbols, phases, MAX_PHASES);
  return true;
}

int GPIOOneWireBus::reset_int() {
  // Wait for the bus to be released, like the bit-bang driver does
  auto pin = static_cast<gpio_num_t>(this->t_pin_->get_pin());
  uint8_t retries = 125;
  while (gpio_get_level(pin) == 0) {
    if (--retries == 0) {
      this->report_(false, LOG_STR("Bus is held low"));
      return -1;
    }
    delayMicroseconds(2);
  }

  rmt_symbol_word_t reset = make_symbol(one_wire_rmt::RESET_LOW_US, one_wire_rmt::RESET_RELEASE_US);
  Phase phases[MAX_PHASES];
  size_t count;
  if (!this->exchange_(this->copy_encoder_, &reset, sizeof(reset), RESET_IDLE_US,
                       one_wire_rmt::RESET_LOW_US + one_wire_rmt::RESET_RELEASE_US, phases, count)) {
    return -1;
  }
  auto result = one_wire_rmt::classify_reset(phases, count);
  this->report_(result != one_wire_rmt::ResetResult::FAULT, LOG_STR("Bus is held low"));
  return static_cast<int>(result);
}

bool GPIOOneWireBus::write_bit_(bool bit) {
  rmt_symbol_word_t slot = write_slot(bit);
  return this->send_(this->copy_encoder_, &slot, sizeof(slot), SLOT_US);
}

void GPIOOneWireBus::write8(uint8_t val) { this->send_(this->bytes_encoder_, &val, sizeof(val), 8 * SLOT_US); }

void GPIOOneWireBus::write64(uint64_t val) { this->send_(this->bytes_encoder_, &val, sizeof(val), 64 * SLOT_US); }

// The functions that read return 0xFF if they fail. That is what an empty bus looks like, and an all-ones
// scratch pad does not pass the CRC check, unlike all zeros.
uint8_t GPIOOneWireBus::read8() {
  static constexpr uint8_t ONES = 0xFF;
  Phase phases[MAX_PHASES];
  size_t count;
  if (!this->exchange_(this->bytes_encoder_, &ONES, sizeof(ONES), SLOT_IDLE_US, 8 * SLOT_US, phases, count))
    return 0xFF;
  uint64_t bits;
  bool ok = one_wire_rmt::decode_bits(phases, count, bits) == 8;
  this->report_(ok, LOG_STR("RMT capture incomplete"));
  return ok ? static_cast<uint8_t>(bits) : 0xFF;
}

uint64_t GPIOOneWireBus::read64() {
  uint64_t ret = 0;
  for (uint8_t i = 0; i < 8; i++) {
    ret |= uint64_t(this->read8()) << (8 * i);
  }
  return ret;
}

uint64_t GPIOOneWireBus::search_int() {
  struct Io {
    GPIOOneWireBus *bus;

    bool search_bits(bool write_first, bool bit, bool &id_bit, bool &cmp_id_bit) {
      rmt_symbol_word_t slots[3];
      size_t slot_count = 0;
      if (write_first)
        slots[slot_count++] = write_slot(bit);
      slots[slot_count++] = read_slot();
      slots[slot_count++] = read_slot();

      Phase phases[MAX_PHASES];
      size_t count;
      if (!this->bus->exchange_(this->bus->copy_encoder_, slots, slot_count * sizeof(slots[0]), SLOT_IDLE_US,
                                slot_count * SLOT_US, phases, count))
        return false;
      uint64_t bits;
      bool ok = one_wire_rmt::decode_bits(phases, count, bits) == slot_count;
      this->bus->report_(ok, LOG_STR("RMT capture incomplete"));
      if (!ok)
        return false;
      bits >>= write_first ? 1 : 0;  // the first pulse is the one written
      id_bit = bits & 1;
      cmp_id_bit = (bits >> 1) & 1;
      return true;
    }

    bool write_bit(bool bit) { return this->bus->write_bit_(bit); }
  } io{this};

  return one_wire_rmt::search_next(io, this->last_discrepancy_, this->last_device_flag_, this->address_);
}

}  // namespace esphome::gpio

#endif  // USE_ONE_WIRE_RMT
