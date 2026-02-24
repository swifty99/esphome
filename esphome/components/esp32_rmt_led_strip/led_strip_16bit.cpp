#include "led_strip_16bit.h"

#ifdef USE_ESP32

#include <esp_idf_version.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>
#include <cstring>
#include <driver/rmt_tx.h>

namespace esphome {
namespace esp32_rmt_led_strip {

static const char *const TAG = "esp32_rmt_led_strip.16bit";

void ESP32RMTLEDStripLightOutput16::setup() {
  // Parent allocates buf_, effect_data_, rmt_buf_, RMT channel + encoder
  ESP32RMTLEDStripLightOutput::setup();

  if (this->is_failed())
    return;

  // Allocate the 16-bit post-gamma buffer (same element count as buf_)
  size_t buffer_size = this->get_buffer_size_();
  RAMAllocator<uint16_t> allocator(this->use_psram_ ? 0 : RAMAllocator<uint16_t>::ALLOC_INTERNAL);
  this->buf_16_ = allocator.allocate(buffer_size);
  if (this->buf_16_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate 16-bit LED buffer!");
    this->mark_failed();
    return;
  }
  memset(this->buf_16_, 0, buffer_size * sizeof(uint16_t));
}

void ESP32RMTLEDStripLightOutput16::setup_state(light::LightState *state) {
  // Store the user-configured gamma value for our 16-bit LUT
  this->gamma_correct_value_ = state->get_gamma_correct();

  // Set the built-in correction to identity gamma (1.0).
  // This means buf_[] will contain brightness-scaled but NOT gamma-corrected values.
  // Brightness scaling (max_brightness * local_brightness) still happens in ESPColorCorrection.
  this->correction_.calculate_gamma_table(1.0f);
  this->state_parent_ = state;

  // Build 16-bit gamma table: maps [0..255] → [0..65535]
  for (uint16_t i = 0; i < 256; i++) {
    float x = i / 255.0f;
    float corrected = powf(x, this->gamma_correct_value_);
    this->gamma_table_16_[i] = static_cast<uint16_t>(roundf(corrected * 65535.0f));
  }

  ESP_LOGD(TAG, "16-bit gamma table built (gamma=%.2f)", this->gamma_correct_value_);
  ESP_LOGD(TAG, "  LUT[1]=%u  LUT[2]=%u  LUT[128]=%u  LUT[255]=%u",
           this->gamma_table_16_[1], this->gamma_table_16_[2],
           this->gamma_table_16_[128], this->gamma_table_16_[255]);
}

void ESP32RMTLEDStripLightOutput16::write_state(light::LightState *state) {
  // Rate limiting
  uint32_t now = micros();
  if (*this->max_refresh_rate_ != 0 && (now - this->last_refresh_) < *this->max_refresh_rate_) {
    this->schedule_show();
    return;
  }
  this->last_refresh_ = now;
  this->mark_shown_();

  ESP_LOGVV(TAG, "Writing 16-bit RGB values to bus");

  esp_err_t error = rmt_tx_wait_all_done(this->channel_, 1000);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "RMT TX timeout");
    this->status_set_warning();
    return;
  }
  delayMicroseconds(50);

  size_t buffer_size = this->get_buffer_size_();

  // Step 1: Apply 16-bit gamma expansion
  //   buf_[i] contains brightness-scaled, linear uint8_t (identity gamma)
  //   gamma_table_16_[buf_[i]] produces the gamma-corrected uint16_t
  for (size_t i = 0; i < buffer_size; i++) {
    this->buf_16_[i] = this->gamma_table_16_[this->buf_[i]];
  }

  // Step 2: Truncate to 8-bit for RMT output (Phase 1)
  // TODO Phase 2: Replace this with temporal dithering
  for (size_t i = 0; i < buffer_size; i++) {
    this->rmt_buf_[i] = static_cast<uint8_t>(this->buf_16_[i] >> 8);
  }

  // Transmit via RMT
  rmt_transmit_config_t config;
  memset(&config, 0, sizeof(config));
  error = rmt_transmit(this->channel_, this->encoder_, this->rmt_buf_, buffer_size, &config);
  if (error != ESP_OK) {
    ESP_LOGE(TAG, "RMT TX error");
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();
}

void ESP32RMTLEDStripLightOutput16::dump_config() {
  ESP32RMTLEDStripLightOutput::dump_config();
  ESP_LOGCONFIG(TAG, "  High Precision: 16-bit");
  ESP_LOGCONFIG(TAG, "  Gamma (16-bit LUT): %.2f", this->gamma_correct_value_);
}

}  // namespace esp32_rmt_led_strip
}  // namespace esphome

#endif  // ESP_IDF_VERSION >= 5.3.0
#endif  // USE_ESP32
