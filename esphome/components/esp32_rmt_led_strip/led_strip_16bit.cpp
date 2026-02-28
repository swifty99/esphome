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
#include <esp_clk_tree.h>

namespace esphome {
namespace esp32_rmt_led_strip {

static const char *const TAG = "esp32_rmt_led_strip.16bit";



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

    uint32_t frame_start_us = micros();

    // Select the back buffer (the one NOT currently being transmitted)
    uint8_t *back_buf = (this->active_buf_ == 0) ? this->rmt_buf_b_ : this->rmt_buf_a_;

    // Check for static scene — if all fractional parts are 0, just truncate once and hold
    if (this->is_static_scene_()) {
      size_t buffer_size = this->get_buffer_size_();
      for (size_t i = 0; i < buffer_size; i++) {
        back_buf[i] = static_cast<uint8_t>(this->buf_16_[i] >> 8);
      }
      // Zero residuals so error-diffusion starts fresh when scene becomes dynamic again
      if (this->residual_ != nullptr) {
        memset(this->residual_, 0, buffer_size * sizeof(int16_t));
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

    // Apply dithering for this frame
    uint32_t dither_start_us = micros();
    this->dither_frame_(back_buf);
    uint32_t dither_us = micros() - dither_start_us;

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

    // Ensure minimum latch delay after TX done before next frame
    if (this->latch_delay_us_ > 0) {
      delayMicroseconds(this->latch_delay_us_);
    }

    // Periodic performance + scene diagnostics (every 256 frames)
    if ((this->frame_counter_ & 0xFF) == 0) {
      uint32_t frame_total_us = micros() - frame_start_us;
      float fps = (frame_total_us > 0) ? 1000000.0f / frame_total_us : 0;

      // Update measured fps and recompute min_frac for flicker cutoff
      this->measured_fps_ = fps;
      if (fps > 0 && this->min_flicker_hz_ > 0) {
        float mf = ceilf((float) this->min_flicker_hz_ * 257.0f / fps);
        this->min_frac_ = (mf > 255.0f) ? 255 : static_cast<uint8_t>(mf);
      } else {
        this->min_frac_ = 0;  // No cutoff if flicker_hz is 0
      }

      // Scene analysis: scan buf_16_ for dithering stats
      uint16_t min_nz = 0xFFFF, max_val = 0;
      uint32_t dithering_count = 0;  // bytes with nonzero frac
      uint32_t cutoff_count = 0;     // bytes suppressed by min_frac cutoff
      for (size_t j = 0; j < buffer_size; j++) {
        uint16_t v = this->buf_16_[j];
        if (v > max_val) max_val = v;
        uint8_t frac = v & 0xFF;
        if (frac != 0) {
          dithering_count++;
          if (frac < this->min_frac_) cutoff_count++;
          if (v < min_nz) min_nz = v;
        }
      }
      uint8_t min_frac = (min_nz != 0xFFFF) ? (min_nz & 0xFF) : 0;
      uint8_t min_high = (min_nz != 0xFFFF) ? (min_nz >> 8) : 0;

      float br_pct = this->brightness_16_ * 100.0f / 65535.0f;
      const char *mode = this->use_dithering_ ? "errordiff" : "off";
      ESP_LOGD(TAG, "Dither perf: dither=%" PRIu32 "us  frame=%" PRIu32 "us  fps=%.0f  mode=%s  latch=%" PRIu32 "us  br=%.1f%%  min_frac=%u  flicker_hz=%u",
               dither_us, frame_total_us, fps, mode, this->latch_delay_us_, br_pct,
               this->min_frac_, this->min_flicker_hz_);
      ESP_LOGD(TAG, "  Scene: %" PRIu32 "/%zu bytes dithered  %" PRIu32 " cutoff  min_nz=0x%04X (high=%u frac=%u)  max=0x%04X",
               dithering_count, buffer_size, cutoff_count,
               (min_nz != 0xFFFF ? min_nz : 0), min_high, min_frac, max_val);

      // Residual stats for error-diffusion mode
      if (this->use_dithering_ && this->residual_ != nullptr) {
        int16_t res_min = 0, res_max = 0;
        int32_t res_sum = 0;
        for (size_t j = 0; j < buffer_size; j++) {
          int16_t r = this->residual_[j];
          if (r < res_min) res_min = r;
          if (r > res_max) res_max = r;
          res_sum += r;
        }
        ESP_LOGD(TAG, "  Residuals: min=%d max=%d avg=%.1f",
                 res_min, res_max, (float) res_sum / (float) buffer_size);
      }
    }
  }

  // Clean exit
  vTaskDelete(nullptr);
}

void ESP32RMTLEDStripLightOutput16::dither_frame_(uint8_t *rmt_buf) {
  size_t buffer_size = this->get_buffer_size_();

  if (!this->use_dithering_) {
    // No dithering — just truncate 16-bit to 8-bit (drop fractional part).
    for (size_t i = 0; i < buffer_size; i++) {
      rmt_buf[i] = static_cast<uint8_t>(this->buf_16_[i] >> 8);
    }
  } else {
    // Fadecandy-style error-diffusion temporal dithering (sigma-delta modulation).
    // Each pixel/channel has a persistent residual that accumulates quantization error.
    // Each frame:
    //   1. Check flicker cutoff: if frac < min_frac_, truncate (no dither)
    //   2. Add previous residual to current 16-bit target
    //   3. Round to nearest 8-bit value with clamping
    //   4. Save new residual = (desired_16 + old_residual) - (output_8 * 257)
    // Over time, the average output converges to the exact 16-bit target.
    // Each pixel accumulates independently — no synchronized flashing.
    uint8_t min_frac = this->min_frac_;
    for (size_t i = 0; i < buffer_size; i++) {
      uint16_t val16 = this->buf_16_[i];
      uint8_t frac = static_cast<uint8_t>(val16 & 0xFF);

      // Flicker cutoff: if the fractional part is too small, the sigma-delta
      // toggle frequency would be below min_flicker_hz_. Just truncate to
      // the integer part and zero the residual to avoid stale error buildup.
      // toggle_freq = fps * frac / 257, so frac < min_frac → too slow.
      if (frac != 0 && frac < min_frac) {
        rmt_buf[i] = static_cast<uint8_t>(val16 >> 8);
        this->residual_[i] = 0;
        continue;
      }

      // Start with 16-bit target and add residual from previous frame
      int32_t val = static_cast<int32_t>(val16) + this->residual_[i];

      // Round to nearest 8-bit value: add 0x80 (half an 8-bit LSB) then shift.
      // Clamp to [0, 0xFFFF] before shifting to handle negative or overflow values.
      int32_t rounded = val + 0x80;
      if (rounded < 0) rounded = 0;
      if (rounded > 0xFFFF) rounded = 0xFFFF;
      uint8_t out8 = static_cast<uint8_t>(rounded >> 8);

      // Store the 8-bit output
      rmt_buf[i] = out8;

      // Compute new residual: what we wanted minus what we actually sent.
      // out8 * 257 expands 8-bit back to 16-bit (0x01 -> 0x0101, 0xFF -> 0xFFFF).
      this->residual_[i] = static_cast<int16_t>(val - (static_cast<int32_t>(out8) * 257));
    }
  }
}

bool ESP32RMTLEDStripLightOutput16::is_static_scene_() const {
  // If dithering disabled, always treat as static — just truncate once and hold
  if (!this->use_dithering_)
    return true;
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

  // Allocate per-pixel-per-channel residual buffer for Fadecandy error diffusion
  RAMAllocator<int16_t> allocator_res(this->use_psram_ ? 0 : RAMAllocator<int16_t>::ALLOC_INTERNAL);
  this->residual_ = allocator_res.allocate(buffer_size);
  if (this->residual_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate residual buffer!");
    this->mark_failed();
    return;
  }
  memset(this->residual_, 0, buffer_size * sizeof(int16_t));

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

  // Register RMT TX done callback (ISR).
  // ESP-IDF requires callbacks to be registered before rmt_enable(),
  // but the parent setup() already called rmt_enable(). Disable first.
  rmt_disable(this->channel_);

  rmt_tx_event_callbacks_t cbs;
  memset(&cbs, 0, sizeof(cbs));
  cbs.on_trans_done = on_trans_done_;
  esp_err_t err = rmt_tx_register_event_callbacks(this->channel_, &cbs, this);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Cannot register RMT TX callback: %d", err);
    this->mark_failed();
    return;
  }

  rmt_enable(this->channel_);

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

void ESP32RMTLEDStripLightOutput16::update_state(light::LightState *state) {
  // CRITICAL: Force the ESPColorCorrection to use full brightness (255).
  // This prevents esp_scale8_twice() from clipping low values in 8-bit space.
  //
  // The standard pipeline does: buf_[] = gamma[scale8(color, brightness)]
  // At brightness=254, scale8(1, 254) = (1*256*255)>>16 = 0 — VALUE LOST!
  // We force brightness=255 so buf_[] gets the raw effect values,
  // then apply brightness ourselves in 16-bit AFTER gamma.
  this->correction_.set_local_brightness(255);

  // Store the real brightness in 16-bit for our write_state() to use
  auto val = state->current_values;
  float brightness = val.get_brightness() * val.get_state();
  this->brightness_16_ = static_cast<uint16_t>(roundf(brightness * 65535.0f));

  // When no effect is active, we must set the LED colors ourselves.
  // The base class update_state() would do this, but we can't call it because
  // it would overwrite local_brightness with the 8-bit scaled value.
  // color_from_light_color_values() returns Color(color_brightness*R, color_brightness*G, ...)
  // WITHOUT brightness/state — those are handled by brightness_16_ in write_state().
  if (!this->is_effect_active()) {
    this->all() = light::color_from_light_color_values(val);
    this->schedule_show();
  }
}

void ESP32RMTLEDStripLightOutput16::setup_state(light::LightState *state) {
  // Store the user-configured gamma value for our 16-bit LUT
  this->gamma_correct_value_ = state->get_gamma_correct();

  // Set the built-in correction to identity gamma (1.0) AND force full brightness.
  // buf_[] will contain RAW effect RGB values — no brightness, no gamma.
  // Brightness is applied in 16-bit space after gamma in write_state().
  this->correction_.calculate_gamma_table(1.0f);
  this->correction_.set_local_brightness(255);
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

  ESP_LOGVV(TAG, "Updating 16-bit target buffer (brightness_16=%u)", this->brightness_16_);

  // Apply 16-bit gamma expansion from buf_[] → buf_16_[]
  // Then apply brightness as a 16-bit multiply to preserve sub-LSB precision.
  //
  // Pipeline: raw_8bit → gamma_16bit[raw] → × brightness_16 → buf_16_
  //
  // This is the key difference from the standard 8-bit path which does:
  //   scale8(raw, brightness) → gamma_8bit[scaled] → output
  // That 8-bit scale destroys low values (e.g., scale8(1, 254) = 0).
  size_t buffer_size = this->get_buffer_size_();
  if (!this->bypass_gamma_) {
    uint16_t br = this->brightness_16_;
    for (size_t i = 0; i < buffer_size; i++) {
      uint32_t gamma_val = this->gamma_table_16_[this->buf_[i]];
      // 16-bit brightness multiply: (gamma_val * brightness) / 65535
      // Use 32-bit intermediate to avoid overflow
      this->buf_16_[i] = static_cast<uint16_t>((gamma_val * br + 32768) / 65535);
    }

    // One-shot pipeline trace (every ~60 write_state calls ≈ 2 sec at 30fps effect rate)
    static uint8_t trace_counter = 0;
    if (++trace_counter >= 60) {
      trace_counter = 0;
      // Sample first LED's R channel (index 0 in buf_[])
      uint8_t raw = this->buf_[0];
      uint16_t gamma_val = this->gamma_table_16_[raw];
      uint16_t result = this->buf_16_[0];
      ESP_LOGD(TAG, "Pipeline: buf[0]=%u → gamma16=%u → ×br(%u)/65535 → buf16=0x%04X (out=%u.%u)",
               raw, gamma_val, br, result, result >> 8, result & 0xFF);
    }
  }
  // When bypass_gamma_ is true, buf_16_[] was written directly by the effect.
  // Reset flag — the diagnostic effect re-sets it every update cycle.
  this->bypass_gamma_ = false;

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
  ESP_LOGCONFIG(TAG, "  Dither mode: Fadecandy error-diffusion (sigma-delta)");
  ESP_LOGCONFIG(TAG, "  Flicker cutoff: %u Hz (min_frac=%u at %.0f fps)",
               this->min_flicker_hz_, this->min_frac_, this->measured_fps_);

  // Calculate theoretical timing from RMT bit params
  uint32_t rmt_freq;
  esp_clk_tree_src_get_freq_hz((soc_module_clk_t) RMT_CLK_SRC_DEFAULT,
                               ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &rmt_freq);
  float ticks_to_us = 1e6f / (float) rmt_freq;
  float bit0_us = (this->params_.bit0.duration0 + this->params_.bit0.duration1) * ticks_to_us;
  float bit1_us = (this->params_.bit1.duration0 + this->params_.bit1.duration1) * ticks_to_us;
  float avg_bit_us = (bit0_us + bit1_us) / 2.0f;
  uint8_t bits_per_led = (this->is_rgbw_ || this->is_wrgb_) ? 32 : 24;
  float data_us = avg_bit_us * bits_per_led * this->num_leds_;
  float reset_us = (this->params_.reset.duration0 + this->params_.reset.duration1) * ticks_to_us;
  float frame_us = data_us + reset_us + (float) this->latch_delay_us_;
  float max_fps = 1e6f / frame_us;

  // Error-diffusion gives full 16-bit precision (256 sub-levels between 8-bit steps)
  // as long as frame rate is high enough to exceed flicker cutoff
  float effective_bits = 16.0f;
  if (this->min_frac_ > 1) {
    // Flicker cutoff removes the lowest sub-levels
    // Remaining sub-levels = 256 - min_frac
    effective_bits = 8.0f + log2f(256.0f - this->min_frac_);
  }

  ESP_LOGCONFIG(TAG, "  Timing estimate (theoretical):");
  ESP_LOGCONFIG(TAG, "    Bit time: %.2f us (avg)  Reset symbol: %.0f us  Latch delay: %" PRIu32 " us",
               avg_bit_us, reset_us, this->latch_delay_us_);
  ESP_LOGCONFIG(TAG, "    Data TX: %.0f us (%u LEDs x %u bits)  Frame: %.0f us",
               data_us, this->num_leds_, bits_per_led, frame_us);
  ESP_LOGCONFIG(TAG, "    Max frame rate: %.0f fps", max_fps);
  ESP_LOGCONFIG(TAG, "    Effective bit depth: %.1f bits", effective_bits);

  // Low-end LUT analysis: show where flicker cutoff suppresses dithering
  ESP_LOGCONFIG(TAG, "  Low-end gamma LUT analysis (input → val16 = high.frac → dither status):");
  int first_8bit_nonzero = -1;
  int first_dithered = -1;
  for (int i = 0; i <= 30; i++) {
    uint16_t v = this->gamma_table_16_[i];
    uint8_t high = v >> 8;
    uint8_t frac = v & 0xFF;
    bool suppressed = (frac != 0 && frac < this->min_frac_);
    float toggle_hz = (this->measured_fps_ > 0) ? (this->measured_fps_ * frac / 257.0f) : 0;
    const char *status = (frac == 0) ? "exact" : (suppressed ? "CUTOFF" : "dither");
    ESP_LOGCONFIG(TAG, "    [%2d] 0x%04X → %3u.%3u  %s  toggle=%.0fHz",
                 i, v, high, frac, status, toggle_hz);
    if (first_8bit_nonzero < 0 && high > 0) first_8bit_nonzero = i;
    if (first_dithered < 0 && frac >= this->min_frac_ && frac != 0) first_dithered = i;
  }
  if (first_dithered >= 0) {
    ESP_LOGCONFIG(TAG, "    → First input with active dithering: %d (frac >= %u → toggle >= %u Hz)",
                 first_dithered, this->min_frac_, this->min_flicker_hz_);
  }
  if (first_8bit_nonzero >= 0) {
    ESP_LOGCONFIG(TAG, "    → First input with 8-bit output >= 1: %d", first_8bit_nonzero);
  }
}

}  // namespace esp32_rmt_led_strip
}  // namespace esphome

#endif  // ESP_IDF_VERSION >= 5.3.0
#endif  // USE_ESP32
