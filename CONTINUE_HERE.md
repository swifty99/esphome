# ESP32 RMT LED Strip — 16-bit Gamma + Temporal Dithering

## Continuation Context

This document captures the full state of the implementation for continuing in a new environment.

---

## 1. Repository & Branch Layout

**Repo**: `c:\repos\esphome\esphome` — clone of `swifty99/esphome` (fork of `esphome/esphome`)

**Remotes**:
- `origin` → `https://github.com/swifty99/esphome` (fork)
- `upstream` → `https://github.com/esphome/esphome` (upstream)

**Branches** (two separate PRs):

```
* d075eda36 (feature/temporal-dithering, origin/feature/temporal-dithering)
|   docs: update PR description for Phase 2 temporal dithering
* 9aa4c04b7
|   feat(esp32_rmt_led_strip): Phase 2 - temporal dithering with 2x2 Bayer matrix
* 55aa30c89 (feature/led-strip-16bit-gamma, origin/feature/led-strip-16bit-gamma)
|   esp32_rmt_led_strip: Add 16-bit high-precision gamma path
* ad2da0af5 (upstream/dev, origin/dev, dev)
    base: upstream dev
```

| Branch | PR Scope | Base |
|---|---|---|
| `feature/led-strip-16bit-gamma` | Phase 1: 16-bit gamma LUT, YAML option, tests | `dev` |
| `feature/temporal-dithering` | Phase 2: temporal dithering (on top of Phase 1) | `feature/led-strip-16bit-gamma` |

---

## 2. Files Changed (vs upstream `dev`)

### Phase 1 — `feature/led-strip-16bit-gamma` (commit `55aa30c89`)

| File | Status | Description |
|---|---|---|
| `esphome/components/esp32_rmt_led_strip/led_strip_16bit.h` | NEW | Phase 1 header (simple, no FreeRTOS) |
| `esphome/components/esp32_rmt_led_strip/led_strip_16bit.cpp` | NEW | Phase 1 impl (truncation >>8) |
| `esphome/components/esp32_rmt_led_strip/light.py` | MODIFIED | +`CONF_HIGH_PRECISION`, +class declaration, +type override |
| `esphome/components/esp32_rmt_led_strip/PR_DESCRIPTION.md` | NEW | PR description (Phase 1 only) |
| `tests/component_tests/esp32_rmt_led_strip/__init__.py` | NEW | Empty init |
| `tests/component_tests/esp32_rmt_led_strip/.gitignore` | NEW | Ignore cache |
| `tests/component_tests/esp32_rmt_led_strip/test_high_precision.py` | NEW | 6 Phase 1 tests |
| `tests/component_tests/esp32_rmt_led_strip/test_high_precision.yaml` | NEW | Test YAML |
| `tests/components/esp32_rmt_led_strip/common.yaml` | MODIFIED | +high_precision entry |
| `tests/components/esp32_rmt_led_strip/test.esp32-idf.yaml` | MODIFIED | +pin3 sub |
| `tests/components/esp32_rmt_led_strip/test.esp32-s3-idf.yaml` | MODIFIED | +pin3 sub |
| `tests/components/esp32_rmt_led_strip/test.esp32-c3-idf.yaml` | MODIFIED | +pin3 sub |

### Phase 2 — `feature/temporal-dithering` (commits `9aa4c04b7` + `d075eda36`)

| File | Status | Description |
|---|---|---|
| `esphome/components/esp32_rmt_led_strip/led_strip_16bit.h` | REWRITTEN | FreeRTOS includes, ISR, dither task, double buffer, semaphores (~93 lines) |
| `esphome/components/esp32_rmt_led_strip/led_strip_16bit.cpp` | REWRITTEN | Full dithering implementation (~285 lines) |
| `tests/component_tests/esp32_rmt_led_strip/test_high_precision.py` | EXPANDED | 9 new dithering tests (15 total) |
| `esphome/components/esp32_rmt_led_strip/PR_DESCRIPTION.md` | UPDATED | Phase 2 docs |

**No original ESPHome files are modified** — `led_strip.h`, `led_strip.cpp`, and all files in `esphome/components/light/` are untouched.

---

## 3. Architecture Overview

### Signal Flow

```
User / Effect → set_red(uint8_t)     ← standard 8-bit interface (unchanged)
  → ESPColorCorrection (gamma=1.0 identity)
    → buf_[i]                         ← uint8_t, brightness-scaled, linear

write_state() [non-blocking]:
  → buf_16_[i] = gamma_table_16_[buf_[i]]   ← 256→uint16_t LUT (real gamma here)
  → xSemaphoreGive(buf_updated_sem_)         ← signal dither task

dither_loop_() [FreeRTOS background task, priority = main+1]:
  → is_static_scene_()? → truncate once, block on semaphore (zero CPU)
  → dither_frame_(back_buf, frame_counter_):
      for each byte i:
        high = buf_16_[i] >> 8
        frac = buf_16_[i] & 0xFF
        threshold = BAYER2X2[frame & 1][(i/bytes_per_led) & 1]
        output = (frac > threshold && high < 255) ? high+1 : high
  → rmt_transmit(back_buf)
  → swap active_buf_ (ping-pong double buffer)
  → xSemaphoreTake(tx_done_sem_)    ← ISR signals when TX complete
  → frame_counter_++, loop
```

### Key Design Decisions

1. **Identity gamma in ESPColorCorrection** → `buf_[]` is linear, brightness-scaled
2. **Real gamma in 16-bit LUT** → `gamma_table_16_[256]` built once in `setup_state()`
3. **2×2 Bayer ordered dithering** → 4 sub-levels, thresholds `{0,64,128,192}`, 2-frame cycle
4. **ISR-driven continuous refresh** → `on_trans_done_` (IRAM_ATTR) gives `tx_done_sem_`
5. **Double-buffered RMT** → `rmt_buf_a_` (reuses parent's `rmt_buf_`), `rmt_buf_b_` (new alloc)
6. **Static scene optimization** → blocks on `buf_updated_sem_` when all fractions = 0
7. **Non-blocking write_state()** → just updates `buf_16_[]` and signals task
8. **ESP-IDF ≥ 5.3 guard** → `#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)`

### Class Hierarchy

```
light::LightOutput
  └── light::AddressableLight
        └── ESP32RMTLEDStripLightOutput          ← existing (UNCHANGED)
              └── ESP32RMTLEDStripLightOutput16   ← NEW (inherits everything)
```

### YAML Configuration

```yaml
light:
  - platform: esp32_rmt_led_strip
    pin: GPIO48
    num_leds: 60
    rgb_order: GRB
    chipset: WS2812
    high_precision: true    # ← enables 16-bit gamma + temporal dithering
```

---

## 4. Bayer Dithering — Algorithm Details

### Threshold Matrix

```
BAYER2X2[2][2] = {
    {0, 128},     // frame 0: even LED=0,   odd LED=128
    {192, 64},    // frame 1: even LED=192, odd LED=64
};
```

Sorted thresholds: `[0, 64, 128, 192]` — spacing of 64 (= 256/4 sub-levels)

### Per-Byte Logic (C++ `dither_frame_()`)

```cpp
uint8_t high = val16 >> 8;       // integer part
uint8_t frac = val16 & 0xFF;     // fractional part (0..255)
uint8_t spatial_idx = (i / bytes_per_led) & 1;  // even/odd LED
uint8_t temporal_idx = frame_index & 1;          // even/odd frame
uint8_t threshold = BAYER2X2[temporal_idx][spatial_idx];
output = (frac > threshold && high < 255) ? high + 1 : high;
```

### Example: `val16 = 0x0A80` (high=10, frac=128)

| Frame | LED | Threshold | frac>thr? | Output |
|---|---|---|---|---|
| 0 | even | 0 | 128>0 ✓ | 11 |
| 0 | odd | 128 | 128>128 ✗ | 10 |
| 1 | even | 192 | 128>192 ✗ | 10 |
| 1 | odd | 64 | 128>64 ✓ | 11 |

→ 2/4 = 50% duty at level 11, average = 10.5 ≈ 10 + 128/256

---

## 5. Test Results

### Python Tests (15/15 passing ✅)

```
pytest tests/component_tests/esp32_rmt_led_strip/test_high_precision.py -v
```

**Phase 1 (6 tests)**:
- `test_high_precision_generates_16bit_class` ✅
- `test_high_precision_false_generates_normal_class` ✅
- `test_high_precision_default_is_false` ✅
- `test_high_precision_can_be_enabled` ✅
- `test_gamma_lut_math` ✅
- `test_gamma_lut_resolution_improvement` ✅

**Phase 2 (9 tests)**:
- `test_bayer_matrix_properties` ✅
- `test_dither_zero_fraction_always_truncates` ✅
- `test_dither_full_fraction_always_rounds_up` ✅
- `test_dither_full_fraction_at_max_clamps` ✅
- `test_dither_sub_levels_count` ✅
- `test_dither_average_converges` ✅
- `test_dither_more_distinct_values_than_truncation` ✅
- `test_static_scene_detection` ✅
- `test_dither_spatial_distribution` ✅

**Regression**: 204/204 component tests pass (1 pre-existing unrelated failure: `test_svg_with_mm_dimensions_succeeds` — missing `resvg_py`)

### Config Validation (3/3 passing ✅)

```
python script/test_build_components.py -c esp32_rmt_led_strip -e config
```

- `esp32-idf` ✅
- `esp32-s3-idf` ✅
- `esp32-c3-idf` ✅

### C++ Code Generation ✅

```
esphome compile --only-generate tests/test_build_components/build/esp32_rmt_led_strip.test.esp32-idf.yaml
```

Generated `main.cpp` correctly produces:
```cpp
static esp32_rmt_led_strip::ESP32RMTLEDStripLightOutput16 *...;
// ...
new esp32_rmt_led_strip::ESP32RMTLEDStripLightOutput16();
```

### Full Compile — NOT YET DONE ❌

PlatformIO compilation was blocked by:
1. SSL certificate errors (corporate proxy) — **fixed** by `pip install pip-system-certs`
2. `uv installation via pip timed out` — PlatformIO ESP-IDF toolchain setup issue
3. `pip install uv` was installed but the compile still timed out on ESP-IDF Python deps

**To complete compilation**, run in an environment with clean internet:

```bash
# Option A: via test script
python script/test_build_components.py -c esp32_rmt_led_strip -t esp32-idf -e compile

# Option B: direct esphome compile
python -m esphome compile tests/components/esp32_rmt_led_strip/test.esp32-idf.yaml

# Option C: direct command (as generated by test script)
python -m esphome -s component_name esp32_rmt_led_strip \
  -s component_dir ../../components/esp32_rmt_led_strip \
  -s test_name test -s target_platform esp32-idf \
  compile tests/test_build_components/build/esp32_rmt_led_strip.test.esp32-idf.yaml
```

---

## 6. Remaining TODO

### Must Do
- [ ] **Full PlatformIO compile** for all 3 targets (esp32-idf, esp32-s3-idf, esp32-c3-idf)
- [ ] **Fix any C++ compile errors** discovered during full build

### Should Do
- [ ] **Hardware test** on real WS2812 strip — verify dithering is visible in low brightness
- [ ] **Verify dump_config()** output shows dithering info
- [ ] **Test RGBW strips** with `high_precision: true`
- [ ] **Test effects** (rainbow, scan, etc.) with dithering active

### Future (Phase 3)
- [ ] **Keyframe interpolation** — Fadecandy-style smooth transitions between `write_state()` calls at dither frame rate

---

## 7. Environment Setup

```powershell
cd c:\repos\esphome\esphome
git checkout feature/temporal-dithering   # has all Phase 1 + Phase 2 code

# Python venv (Python 3.12.8)
.\venv\Scripts\activate
# or recreate:
python -m venv venv
.\venv\Scripts\activate
pip install -e ".[dev,test]" --index-url https://pypi.org/simple/
pip install pip-system-certs   # fixes SSL behind corporate proxy
pip install uv                 # needed by PlatformIO for ESP-IDF

# Run Python tests
python -m pytest tests/component_tests/esp32_rmt_led_strip/test_high_precision.py -v

# Run all component tests (regression check)
python -m pytest tests/component_tests/ -v --tb=short

# Config validation
python script/test_build_components.py -c esp32_rmt_led_strip -e config

# Full compile (needs PlatformIO + toolchains)
python script/test_build_components.py -c esp32_rmt_led_strip -e compile
```

---

## 8. Key Source Files — Quick Reference

### `light.py` (196 lines) — Modified

Key additions (search for `HIGH_PRECISION`):
- Line ~35: `ESP32RMTLEDStripLightOutput16` class declaration
- Line ~85: `CONF_HIGH_PRECISION = "high_precision"` 
- Line ~139: `cv.Optional(CONF_HIGH_PRECISION, default=False): cv.boolean`
- Line ~153: Type override in `to_code()`:
  ```python
  if config[CONF_HIGH_PRECISION]:
      config[CONF_OUTPUT_ID].type = ESP32RMTLEDStripLightOutput16
  ```

### `led_strip_16bit.h` (93 lines) — New

Class `ESP32RMTLEDStripLightOutput16 : public ESP32RMTLEDStripLightOutput`

Key members:
- `buf_16_` — uint16_t post-gamma buffer
- `rmt_buf_a_`, `rmt_buf_b_` — double-buffered RMT output
- `gamma_table_16_[256]` — 16-bit gamma LUT
- `tx_done_sem_`, `buf_updated_sem_` — FreeRTOS semaphores
- `dither_task_handle_` — background task handle
- `frame_counter_` — Bayer temporal index
- `on_trans_done_()` — IRAM_ATTR ISR

### `led_strip_16bit.cpp` (285 lines) — New

Key functions:
- `on_trans_done_()` — ISR, gives `tx_done_sem_` from ISR context
- `dither_task_fn_()` → `dither_loop_()` — background continuous refresh
- `dither_frame_(rmt_buf, frame_index)` — applies 2×2 Bayer per byte
- `is_static_scene_()` — checks all fractional parts == 0
- `setup()` — parent setup + alloc buf_16_, rmt_buf_b_, semaphores, register ISR, create task
- `setup_state()` — capture gamma, set identity correction, build 16-bit LUT
- `write_state()` — non-blocking: gamma expand buf_→buf_16_, signal task
- `dump_config()` — logs "16-bit with temporal dithering", gamma, dither matrix info

### `test_high_precision.py` (380 lines) — New

15 tests covering:
- Codegen (generate_main → class name in output)
- Schema (default false, accepts true)
- Gamma LUT math (monotonic, boundaries, resolution vs 8-bit)
- Bayer matrix properties (4 distinct, evenly spaced)
- Dither correctness (zero frac, full frac, max clamp)
- Dither duty cycle (sub-level count at 50%)
- Average convergence (within ±1.0 of ideal)
- Distinct levels (more than truncation)
- Static scene detection
- Spatial distribution (adjacent LEDs differ)

---

## 9. Potential Issues to Watch

1. **`rmt_tx_register_event_callbacks`** — This API may conflict with the base class's blocking `rmt_tx_wait_all_done()` if both are called. The 16-bit class overrides `write_state()` completely and never calls the parent's `write_state()`, so this should be fine, but verify on hardware.

2. **`trans_queue_depth = 1`** — The base class sets `rmt_tx_channel_config_t.trans_queue_depth = 1`. This means only 1 transaction can be queued. The dither loop waits for `tx_done_sem_` before submitting the next, so this is safe.

3. **Task stack size (4096)** — Should be sufficient for the dither loop (no recursion, small locals). Monitor with `uxTaskGetStackHighWaterMark()` if needed.

4. **`rmt_buf_a_` reuse** — We reuse the parent's `rmt_buf_` as `rmt_buf_a_`. The parent never touches it after `setup()` since we override `write_state()`. Safe but worth noting.

5. **PSRAM latency** — If `use_psram: true` and the buffers are in PSRAM, the dither loop's per-byte access may be slower. For 300 RGB LEDs (900 bytes), this is ~10µs even with PSRAM.

6. **Thread safety of `buf_16_[]`** — `write_state()` writes `buf_16_[]` from the main loop; `dither_loop_()` reads it from the task. There's no mutex — a torn read could cause a single frame glitch (one byte at old value). This is acceptable for LED output (imperceptible). If strictness is needed, add a mutex around the buffer copy.
