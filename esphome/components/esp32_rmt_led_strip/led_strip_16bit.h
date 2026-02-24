#pragma once

#ifdef USE_ESP32

#include "led_strip.h"

#include <esp_idf_version.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)

namespace esphome {
namespace esp32_rmt_led_strip {

/// 16-bit high-precision variant of the RMT LED strip output.
///
/// The input interface remains 8-bit (ESPColorView / effects are fully compatible).
/// The built-in ESPColorCorrection runs with identity gamma (gamma = 1.0) so that
/// buf_[] stores brightness-scaled, linear (non-gamma-corrected) uint8_t values.
///
/// In write_state(), a 16-bit gamma LUT expands each buf_[] byte to uint16_t,
/// preserving much finer resolution in the dark end of the curve.
/// Phase 1 truncates (>>8) to 8-bit for the RMT encoder.
/// Phase 2 will add temporal dithering at this point.
class ESP32RMTLEDStripLightOutput16 : public ESP32RMTLEDStripLightOutput {
 public:
  void setup() override;
  void write_state(light::LightState *state) override;
  void setup_state(light::LightState *state) override;

  void dump_config() override;

 protected:
  /// 16-bit post-gamma buffer, same element count as buf_[]
  uint16_t *buf_16_{nullptr};

  /// 256-entry LUT: maps pre-gamma uint8_t → post-gamma uint16_t (512 bytes)
  uint16_t gamma_table_16_[256];

  /// Stored real gamma exponent (the one the user configured)
  float gamma_correct_value_{2.8f};
};

}  // namespace esp32_rmt_led_strip
}  // namespace esphome

#endif  // ESP_IDF_VERSION >= 5.3.0
#endif  // USE_ESP32
