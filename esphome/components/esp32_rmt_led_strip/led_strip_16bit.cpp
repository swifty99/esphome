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

// 2×2 ordered Bayer matrix, scaled to 0..255 range.
// Temporal index = frame_counter & 1  (2 rows → cycle every 2 frames)
// Spatial index  = (led_byte_index / bytes_per_led) & 1
// Threshold layout:
//   [ 0, 128 ]    →  frame 0: even LEDs threshold=0,   odd LEDs threshold=128
//   [ 192, 64 ]   →  frame 1: even LEDs threshold=192, odd LEDs threshold=64
// This gives 4 perceptual sub-levels between adjacent 8-bit values.
static const uint8_t BAYER2X2[2][2] = {
    {0, 128},
    {192, 64},
};

// ─── ISR callback (IRAM) ───────────────────────────────────────────────

bool IRAM_ATTR ESP32RMTLEDStripLightOutput16::on_trans_done_(
    rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_ctx) {
  auto *self = static_cast<ESP32RMTLEDStripLightOutput16 *>(user_ctx);
  BaseType_t high_task_woken = pdFALSE;
  xSemaphoreGiveFromISR(self->tx_done_sem_, &high_task_woken);
  return high_task_woken == pdTRUE;
}

// ─── Dither task ───────────────────────────────────────────────────────

void ESP32RMTLEDStripLightOutput16::dither_task_fn_(void *arg) {
  auto *self = static_cast<ESP32RMTLEDStripLightOutput16 *>(arg);
  self->dither_loop_();
}

void ESP32RMTLEDStripLightOutput16::dither_loop_() {
  // Wait for the first write_state() before starting to transmit
  while (!this->first_write_done_ && this->dither_task_running_) {
    xSemaphoreTake(this->buf_updated_sem_, pdMS_TO_TICKS(100));
  }

  while (this->dither_task_running_) {
    // Check if new target data is available (non-blocking poll)
    // This just clears the semaphore — buf_16_ is already written by write_state()
    xSemaphoreTake(this->buf_updated_sem_, 0);

    // Select the back buffer (the one NOT currently being transmitted)
    uint8_t *back_buf = (this->active_buf_ == 0) ? this->rmt_buf_b_ : this->rmt_buf_a_;

    // Check for static scene — if all fractional parts are 0, just truncate once and hold
    if (this->is_static_scene_()) {
      size_t buffer_size = this->get_buffer_size_();
      for (size_t i = 0; i < buffer_size; i++) {
        back_buf[i] = static_cast<uint8_t>(this->buf_16_[i] >> 8);
      }
      // Transmit the static frame
      rmt_transmit_config_t tx_config;
      memset(&tx_config, 0, sizeof(tx_config));
      esp_err_t err = rmt_transmit(this->channel_, this->encoder_, back_buf, buffer_size, &tx_config);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT TX error (static)");
      }
      // Swap active buffer
      this->active_buf_ ^= 1;

      // Wait for TX done
      xSemaphoreTake(this->tx_done_sem_, pdMS_TO_TICKS(1000));

      // Now wait for new data — no point dithering a static scene
      // This blocks until write_state() provides new target values
      xSemaphoreTake(this->buf_updated_sem_, portMAX_DELAY);
      continue;
    }

    // Apply Bayer dithering for this frame
    this->dither_frame_(back_buf, this->frame_counter_);

    // Transmit
    size_t buffer_size = this->get_buffer_size_();
    rmt_transmit_config_t tx_config;
    memset(&tx_config, 0, sizeof(tx_config));
    esp_err_t err = rmt_transmit(this->channel_, this->encoder_, back_buf, buffer_size, &tx_config);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "RMT TX error (dither)");
    }

    // Swap active buffer
    this->active_buf_ ^= 1;
    this->frame_counter_++;

    // Wait for TX done before preparing next frame
    if (xSemaphoreTake(this->tx_done_sem_, pdMS_TO_TICKS(1000)) != pdTRUE) {
      ESP_LOGW(TAG, "RMT TX done timeout");
    }
  }

  // Clean exit
  vTaskDelete(nullptr);
}

void ESP32RMTLEDStripLightOutput16::dither_frame_(uint8_t *rmt_buf, uint8_t frame_index) {
  size_t buffer_size = this->get_buffer_size_();
  uint8_t temporal_idx = frame_index & 1;
  size_t bytes_per_led = (this->is_rgbw_ || this->is_wrgb_) ? 4 : 3;

  for (size_t i = 0; i < buffer_size; i++) {
    uint16_t val16 = this->buf_16_[i];
    uint8_t high = static_cast<uint8_t>(val16 >> 8);
    uint8_t frac = static_cast<uint8_t>(val16 & 0xFF);

    // Spatial index: which LED this byte belongs to
    uint8_t spatial_idx = (i / bytes_per_led) & 1;

    uint8_t threshold = BAYER2X2[temporal_idx][spatial_idx];

    // If fractional part exceeds the Bayer threshold, round up
    if (frac > threshold && high < 255) {
      rmt_buf[i] = high + 1;
    } else {
      rmt_buf[i] = high;
    }
  }
}

bool ESP32RMTLEDStripLightOutput16::is_static_scene_() const {
  size_t buffer_size = this->get_buffer_size_();
  for (size_t i = 0; i < buffer_size; i++) {
    if ((this->buf_16_[i] & 0xFF) != 0) {
      return false;
    }
  }
  return true;
}

// ─── Overridden lifecycle methods ──────────────────────────────────────

void ESP32RMTLEDStripLightOutput16::setup() {
  // Parent allocates buf_, effect_data_, rmt_buf_, RMT channel + encoder
  ESP32RMTLEDStripLightOutput::setup();

  if (this->is_failed())
    return;

  size_t buffer_size = this->get_buffer_size_();

  // Allocate the 16-bit post-gamma buffer
  RAMAllocator<uint16_t> allocator16(this->use_psram_ ? 0 : RAMAllocator<uint16_t>::ALLOC_INTERNAL);
  this->buf_16_ = allocator16.allocate(buffer_size);
  if (this->buf_16_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate 16-bit LED buffer!");
    this->mark_failed();
    return;
  }
  memset(this->buf_16_, 0, buffer_size * sizeof(uint16_t));

  // Allocate double buffers for RMT output (parent already allocated rmt_buf_, reuse as buf A)
  this->rmt_buf_a_ = this->rmt_buf_;  // Reuse parent's allocation
  RAMAllocator<uint8_t> allocator8(this->use_psram_ ? 0 : RAMAllocator<uint8_t>::ALLOC_INTERNAL);
  this->rmt_buf_b_ = allocator8.allocate(buffer_size);
  if (this->rmt_buf_b_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate RMT double buffer!");
    this->mark_failed();
    return;
  }
  memset(this->rmt_buf_b_, 0, buffer_size);

  // Create semaphores
  this->tx_done_sem_ = xSemaphoreCreateBinary();
  this->buf_updated_sem_ = xSemaphoreCreateBinary();
  if (this->tx_done_sem_ == nullptr || this->buf_updated_sem_ == nullptr) {
    ESP_LOGE(TAG, "Cannot create semaphores!");
    this->mark_failed();
    return;
  }

  // Register RMT TX done callback (ISR)
  rmt_tx_event_callbacks_t cbs;
  memset(&cbs, 0, sizeof(cbs));
  cbs.on_trans_done = on_trans_done_;
  esp_err_t err = rmt_tx_register_event_callbacks(this->channel_, &cbs, this);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Cannot register RMT TX callback: %d", err);
    this->mark_failed();
    return;
  }

  // Create dither task — priority 1 above the main ESPHome loop task
  this->dither_task_running_ = true;
  UBaseType_t main_prio = uxTaskPriorityGet(nullptr);  // current task = main loop
  BaseType_t result = xTaskCreatePinnedToCore(
      dither_task_fn_,
      "led_dither",
      4096,            // Stack size (bytes)
      this,            // Parameter
      main_prio + 1,   // Priority: slightly above main loop
      &this->dither_task_handle_,
      xPortGetCoreID()  // Same core as setup() runs on
  );
  if (result != pdPASS || this->dither_task_handle_ == nullptr) {
    ESP_LOGE(TAG, "Cannot create dither task!");
    this->dither_task_running_ = false;
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "Dither task created (priority %" PRIu32 ", core %d)",
           (uint32_t)(main_prio + 1), xPortGetCoreID());
}

void ESP32RMTLEDStripLightOutput16::setup_state(light::LightState *state) {
  // Store the user-configured gamma value for our 16-bit LUT
  this->gamma_correct_value_ = state->get_gamma_correct();

  // Set the built-in correction to identity gamma (1.0).
  // This means buf_[] will contain brightness-scaled but NOT gamma-corrected values.
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
  // Rate limiting (same as base class, but for user update rate, not dither rate)
  uint32_t now = micros();
  if (*this->max_refresh_rate_ != 0 && (now - this->last_refresh_) < *this->max_refresh_rate_) {
    this->schedule_show();
    return;
  }
  this->last_refresh_ = now;
  this->mark_shown_();

  ESP_LOGVV(TAG, "Updating 16-bit target buffer");

  // Apply 16-bit gamma expansion from buf_[] → buf_16_[]
  size_t buffer_size = this->get_buffer_size_();
  for (size_t i = 0; i < buffer_size; i++) {
    this->buf_16_[i] = this->gamma_table_16_[this->buf_[i]];
  }

  // Signal the dither task that new target data is available
  this->first_write_done_ = true;
  xSemaphoreGive(this->buf_updated_sem_);

  // Non-blocking return — the dither task handles transmission
}

float ESP32RMTLEDStripLightOutput16::get_setup_priority() const { return setup_priority::HARDWARE; }

void ESP32RMTLEDStripLightOutput16::dump_config() {
  ESP32RMTLEDStripLightOutput::dump_config();
  ESP_LOGCONFIG(TAG, "  High Precision: 16-bit with temporal dithering");
  ESP_LOGCONFIG(TAG, "  Gamma (16-bit LUT): %.2f", this->gamma_correct_value_);
  ESP_LOGCONFIG(TAG, "  Dither matrix: 2x2 Bayer (4 sub-levels, 2-frame cycle)");
}

}  // namespace esp32_rmt_led_strip
}  // namespace esphome

#endif  // ESP_IDF_VERSION >= 5.3.0
#endif  // USE_ESP32
