#pragma once

#ifdef USE_ESP32

#include "led_strip.h"

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
/// The built-in ESPColorCorrection runs with identity gamma (gamma = 1.0) so that
/// buf_[] stores brightness-scaled, linear (non-gamma-corrected) uint8_t values.
///
/// Temporal dithering (Phase 2):
///   A background FreeRTOS task continuously refreshes the strip at maximum rate.
///   Each frame, a 4×4 ordered Bayer matrix selects between adjacent 8-bit output
///   levels based on the fractional part of the 16-bit gamma-corrected value.
///   This gives 16 perceptual sub-levels per 8-bit step without error accumulation
///   or low-frequency flicker (cycle = 4 frames → ≥25 Hz at any practical strip length).
///
///   write_state() is non-blocking: it updates buf_16_[] and signals the dither task.
///   The ISR on RMT TX done wakes the task for the next frame.
class ESP32RMTLEDStripLightOutput16 : public ESP32RMTLEDStripLightOutput {
 public:
  void setup() override;
  void write_state(light::LightState *state) override;
  void setup_state(light::LightState *state) override;
  float get_setup_priority() const override;

  void dump_config() override;

 protected:
  /// Dither task entry point (static, dispatches to instance method)
  static void dither_task_fn_(void *arg);

  /// The actual dither loop running in task context
  void dither_loop_();

  /// Apply 4×4 Bayer dithering from buf_16_[] → rmt_buf (one frame)
  void dither_frame_(uint8_t *rmt_buf, uint8_t frame_index);

  /// Check if all 16-bit values have zero fractional part (no dithering needed)
  bool is_static_scene_() const;

  /// ISR callback for RMT TX done
  static bool IRAM_ATTR on_trans_done_(rmt_channel_handle_t channel,
                                       const rmt_tx_done_event_data_t *edata,
                                       void *user_ctx);

  /// 16-bit post-gamma buffer, same element count as buf_[]
  uint16_t *buf_16_{nullptr};

  /// Double-buffered RMT output (ping-pong)
  uint8_t *rmt_buf_a_{nullptr};
  uint8_t *rmt_buf_b_{nullptr};
  uint8_t active_buf_{0};  // 0 = A transmitting, 1 = B transmitting

  /// 256-entry LUT: maps pre-gamma uint8_t → post-gamma uint16_t (512 bytes)
  uint16_t gamma_table_16_[256];

  /// Stored real gamma exponent (the one the user configured)
  float gamma_correct_value_{2.8f};

  /// Dither task handle and synchronization
  TaskHandle_t dither_task_handle_{nullptr};
  SemaphoreHandle_t tx_done_sem_{nullptr};   // ISR → task: TX finished
  SemaphoreHandle_t buf_updated_sem_{nullptr}; // write_state → task: new data available

  /// Frame counter for Bayer matrix temporal indexing
  uint8_t frame_counter_{0};

  /// Flag: write_state has been called at least once (don't transmit before first data)
  volatile bool first_write_done_{false};

  /// Flag: dither task should exit (for clean shutdown)
  volatile bool dither_task_running_{false};
};

}  // namespace esp32_rmt_led_strip
}  // namespace esphome

#endif  // ESP_IDF_VERSION >= 5.3.0
#endif  // USE_ESP32
