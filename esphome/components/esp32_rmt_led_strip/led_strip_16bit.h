#pragma once

#ifdef USE_ESP32

#include "led_strip.h"

#include <cmath>
#include <esp_idf_version.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace esphome {
namespace esp32_rmt_led_strip {

/// 16-bit high-precision variant of the RMT LED strip output.
///
/// The input interface remains 8-bit (ESPColorView / effects are fully compatible).
/// The built-in ESPColorCorrection runs with identity gamma (gamma = 1.0) AND
/// forced local_brightness = 255, so that buf_[] stores RAW effect RGB values
/// without any brightness scaling or gamma — maximum precision is preserved for
/// the 16-bit gamma LUT.
///
/// The light's brightness is applied as a 16-bit multiply AFTER gamma correction,
/// which preserves sub-LSB precision that would otherwise be destroyed by
/// esp_scale8_twice() in 8-bit space.
///
/// Temporal dithering (Phase 2):
///   A background FreeRTOS task continuously refreshes the strip at maximum rate.
///   Uses Fadecandy-style error diffusion (sigma-delta modulation): each pixel/channel
///   has a persistent residual (error accumulator). Each frame, the residual from the
///   previous frame is added to the 16-bit target, rounded to 8-bit, and the
///   quantization error is saved back. This converges to the exact 16-bit average
///   over time, with no synchronized flashing between pixels.
///
///   Flicker cutoff: at very low 16-bit values (e.g., 0x0005), the sigma-delta only
///   fires every ~50 frames = ~8 Hz, producing visible blinking. The min_flicker_hz_
///   setting defines the minimum acceptable toggle frequency. Channels whose dither
///   frequency would fall below this are clamped to the truncated value (OFF for
///   high=0), eliminating low-frequency flicker at the cost of losing the dimmest
///   sub-levels.
///
///   write_state() is non-blocking: it updates buf_16_[] and signals the dither task.
///   The ISR on RMT TX done wakes the task for the next frame.
class ESP32RMTLEDStripLightOutput16 : public ESP32RMTLEDStripLightOutput {
 public:
  void setup() override;
  void write_state(light::LightState *state) override;
  void setup_state(light::LightState *state) override;
  void update_state(light::LightState *state) override;
  float get_setup_priority() const override;

  void dump_config() override;

  /// Direct access to 16-bit buffer for diagnostic effects (bypasses gamma LUT)
  uint16_t *get_buf_16() { return this->buf_16_; }
  /// Signal the dither task that buf_16_ was updated externally
  void signal_buf_updated() { this->first_write_done_ = true; xSemaphoreGive(this->buf_updated_sem_); }
  /// Skip gamma LUT in write_state (for direct buf_16_ writes)
  void set_bypass_gamma(bool bypass) { this->bypass_gamma_ = bypass; }

 protected:
  /// Dither task entry point (static, dispatches to instance method)
  static void dither_task_fn_(void *arg);

  /// The actual dither loop running in task context
  void dither_loop_();

  /// Apply Fadecandy error-diffusion dithering from buf_16_[] → rmt_buf (one frame)
  void dither_frame_(uint8_t *rmt_buf);

  /// Check if all 16-bit values have zero fractional part (no dithering needed)
  bool is_static_scene_() const;

  /// ISR callback for RMT TX done
  static bool IRAM_ATTR on_trans_done_(rmt_channel_handle_t channel,
                                       const rmt_tx_done_event_data_t *edata,
                                       void *user_ctx);

  /// 16-bit post-gamma buffer, same element count as buf_[]
  uint16_t *buf_16_{nullptr};

  /// Per-pixel-per-channel residual (error accumulator) for Fadecandy-style
  /// sigma-delta temporal dithering. Same element count as buf_16_[].
  /// Stores the quantization error from the previous frame so it can be
  /// added to the next frame's value, ensuring the time-averaged output
  /// converges to the exact 16-bit target.
  int16_t *residual_{nullptr};

  /// Double-buffered RMT output (ping-pong)
  uint8_t *rmt_buf_a_{nullptr};
  uint8_t *rmt_buf_b_{nullptr};
  uint8_t active_buf_{0};  // 0 = A transmitting, 1 = B transmitting

  /// 256-entry LUT: maps pre-gamma uint8_t → post-gamma uint16_t (512 bytes)
  uint16_t gamma_table_16_[256];

  /// Stored real gamma exponent (the one the user configured)
  float gamma_correct_value_{2.8f};

  /// 16-bit brightness from LightState (0 = off, 65535 = full)
  /// Applied as a multiply AFTER 16-bit gamma LUT, preserving sub-LSB precision
  uint16_t brightness_16_{65535};

  /// When true, write_state() skips gamma LUT (buf_16_ was written directly)
  bool bypass_gamma_{false};

  /// Dither task handle and synchronization
  TaskHandle_t dither_task_handle_{nullptr};
  SemaphoreHandle_t tx_done_sem_{nullptr};   // ISR → task: TX finished
  SemaphoreHandle_t buf_updated_sem_{nullptr}; // write_state → task: new data available

  /// Frame counter for diagnostics / perf logging
  uint8_t frame_counter_{0};

  /// Master dithering switch. When false, just truncate 16→8 bit.
  /// When true, Fadecandy error-diffusion is active.
  bool use_dithering_{true};

  /// Minimum acceptable dither toggle frequency in Hz.
  /// Channels whose sigma-delta toggle rate would be below this
  /// are clamped to the truncated 8-bit value (no dithering).
  /// This eliminates visible low-frequency blinking at ultra-low levels.
  /// Formula: toggle_freq = fps * frac / 257
  /// So min_frac = ceil(min_flicker_hz * 257 / fps)
  uint16_t min_flicker_hz_{50};

  /// Computed minimum fractional value for dithering.
  /// Channels with frac < min_frac_ are truncated instead of dithered.
  /// Updated periodically from measured fps and min_flicker_hz_.
  uint8_t min_frac_{30};  // Default for ~433fps, 50Hz cutoff

  /// Last measured fps for min_frac computation
  float measured_fps_{433.0f};

 public:
  void set_use_dithering(bool use_dithering) { this->use_dithering_ = use_dithering; }
  void set_min_flicker_hz(uint16_t hz) {
    this->min_flicker_hz_ = hz;
    // Recompute min_frac from current fps
    if (this->measured_fps_ > 0) {
      float mf = ceilf((float) hz * 257.0f / this->measured_fps_);
      this->min_frac_ = (mf > 255.0f) ? 255 : static_cast<uint8_t>(mf);
    }
  }

 protected:
  /// Flag: write_state has been called at least once (don't transmit before first data)
  volatile bool first_write_done_{false};

  /// Flag: dither task should exit (for clean shutdown)
  volatile bool dither_task_running_{false};
};

}  // namespace esp32_rmt_led_strip
}  // namespace esphome

#endif  // ESP_IDF_VERSION >= 5.3.0
#endif  // USE_ESP32
