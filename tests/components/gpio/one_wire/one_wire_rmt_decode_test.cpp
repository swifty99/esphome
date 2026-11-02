#include <gtest/gtest.h>

#include <algorithm>
#include <set>
#include <vector>

#include "../common.h"

namespace esphome::gpio::testing {

TEST(OneWireRmtFlatten, DropsEmptyPhasesAndMergesNeighbours) {
  // The capture ends with a zero length phase, and the low pulses of two symbols touch
  auto phases = phases_of({{6, 0, 30, 1}, {5, 1, 20, 0}, {4, 0, 0, 1}});
  ASSERT_EQ(phases.size(), 3u);
  EXPECT_EQ(phases[0].level, 0);
  EXPECT_EQ(phases[0].duration, 6u);
  EXPECT_EQ(phases[1].level, 1);
  EXPECT_EQ(phases[1].duration, 35u);
  EXPECT_EQ(phases[2].level, 0);
  EXPECT_EQ(phases[2].duration, 24u);
}

TEST(OneWireRmtFlatten, StopsAtTheEndOfTheOutputBuffer) {
  std::vector<Symbol> symbols(4, {6, 0, 64, 1});
  Phase out[3];
  EXPECT_EQ(flatten(symbols.data(), symbols.size(), out, 3), 3u);
}

TEST(OneWireRmtDecodeBits, ShortPulsesAreOnesAndLongPulsesAreZeros) {
  // LSB first: 1, 0, 1, 1, 0, 0, 0, 1
  std::vector<uint32_t> lows = {6, 40, 7, 8, 15, 60, 20, 6};
  std::vector<Symbol> symbols;
  for (auto low : lows)
    symbols.push_back({low, 0, 70 - low, 1});
  uint64_t bits;
  auto phases = phases_of(symbols);
  EXPECT_EQ(decode_bits(phases.data(), phases.size(), bits), 8u);
  EXPECT_EQ(bits, 0b10001101u);
}

TEST(OneWireRmtDecodeBits, ThresholdIsBetweenTheMasterPulseAndTheMinimumHoldTime) {
  uint64_t bits;
  Phase one[] = {{0, READ_THRESHOLD_US}};
  Phase zero[] = {{0, READ_THRESHOLD_US + 1}};
  EXPECT_EQ(decode_bits(one, 1, bits), 1u);
  EXPECT_EQ(bits, 1u);
  EXPECT_EQ(decode_bits(zero, 1, bits), 1u);
  EXPECT_EQ(bits, 0u);
  // A real master pulse with a slow rise stays on the safe side, and so does a device that lets go early
  EXPECT_LT(READ_LOW_US + 4, READ_THRESHOLD_US);
  EXPECT_GT(15 - 3, READ_THRESHOLD_US);
}

TEST(OneWireRmtDecodeBits, IgnoresHighPhasesAndCountsOnlyLowPulses) {
  uint64_t bits;
  // The capture may start with the bus high
  Phase phases[] = {{1, 100}, {0, 6}, {1, 64}, {0, 30}, {1, 40}};
  EXPECT_EQ(decode_bits(phases, 5, bits), 2u);
  EXPECT_EQ(bits, 1u);
}

TEST(OneWireRmtDecodeBits, DecodesNoMoreThan64Pulses) {
  std::vector<Phase> phases;
  for (int i = 0; i < 70; i++) {
    phases.push_back({0, 6});
    phases.push_back({1, 64});
  }
  uint64_t bits;
  EXPECT_EQ(decode_bits(phases.data(), phases.size(), bits), 64u);
  EXPECT_EQ(bits, ~uint64_t(0));
}

TEST(OneWireRmtSearchBits, SplitsTheReadSlotsOfASearchStep) {
  bool id_bit, cmp_id_bit;
  // Two read slots: the first pulse is the address bit, the second its complement
  EXPECT_TRUE(split_search_bits(0b01, 2, false, id_bit, cmp_id_bit));
  EXPECT_TRUE(id_bit);
  EXPECT_FALSE(cmp_id_bit);
  EXPECT_TRUE(split_search_bits(0b10, 2, false, id_bit, cmp_id_bit));
  EXPECT_FALSE(id_bit);
  EXPECT_TRUE(cmp_id_bit);
  // With a write slot in front, its pulse is skipped, whatever it decoded to
  EXPECT_TRUE(split_search_bits(0b101, 3, true, id_bit, cmp_id_bit));
  EXPECT_FALSE(id_bit);
  EXPECT_TRUE(cmp_id_bit);
  EXPECT_TRUE(split_search_bits(0b010, 3, true, id_bit, cmp_id_bit));
  EXPECT_TRUE(id_bit);
  EXPECT_FALSE(cmp_id_bit);
}

TEST(OneWireRmtSearchBits, RejectsAnIncompleteCapture) {
  bool id_bit, cmp_id_bit;
  EXPECT_FALSE(split_search_bits(0, 1, false, id_bit, cmp_id_bit));
  EXPECT_FALSE(split_search_bits(0, 2, true, id_bit, cmp_id_bit));
  EXPECT_FALSE(split_search_bits(0, 3, false, id_bit, cmp_id_bit));
}

TEST(OneWireRmtReset, DetectsPresence) {
  // reset pulse, 40 µs of waiting, 120 µs presence pulse, then idle
  auto phases = phases_of({{480, 0, 40, 1}, {120, 0, 300, 1}});
  EXPECT_EQ(classify_reset(phases.data(), phases.size()), ResetResult::RESET_RESULT_PRESENT);
}

TEST(OneWireRmtReset, NoDeviceWhenTheBusStaysHigh) {
  auto phases = phases_of({{480, 0, 600, 1}});
  EXPECT_EQ(classify_reset(phases.data(), phases.size()), ResetResult::RESET_RESULT_NO_DEVICE);
}

TEST(OneWireRmtReset, ShortOrLateLowPulsesAreNoPresence) {
  auto too_short = phases_of({{480, 0, 40, 1}, {PRESENCE_LOW_MIN_US - 1, 0, 300, 1}});
  EXPECT_EQ(classify_reset(too_short.data(), too_short.size()), ResetResult::RESET_RESULT_NO_DEVICE);
  auto too_late = phases_of({{480, 0, PRESENCE_WAIT_MAX_US + 1, 1}, {100, 0, 300, 1}});
  EXPECT_EQ(classify_reset(too_late.data(), too_late.size()), ResetResult::RESET_RESULT_NO_DEVICE);
}

TEST(OneWireRmtReset, ABusThatIsHeldLowIsAFault) {
  auto stuck = phases_of({{600, 0, 0, 1}});
  EXPECT_EQ(classify_reset(stuck.data(), stuck.size()), ResetResult::RESET_RESULT_FAULT);
}

TEST(OneWireRmtReset, NoLowPulseAtAllIsAFault) {
  auto none = phases_of({{600, 1, 0, 0}});
  EXPECT_EQ(classify_reset(none.data(), none.size()), ResetResult::RESET_RESULT_FAULT);
  EXPECT_EQ(classify_reset(nullptr, 0), ResetResult::RESET_RESULT_FAULT);
}

TEST(OneWireRmtSearch, FindsNothingOnAnEmptyBus) { EXPECT_TRUE(enumerate({}).empty()); }

TEST(OneWireRmtSearch, FindsASingleDevice) {
  EXPECT_EQ(enumerate({0x120000065C17EE28ULL}), (std::vector<uint64_t>{0x120000065C17EE28ULL}));
}

TEST(OneWireRmtSearch, FindsTwoRealDevices) {
  // The ROM codes of the two DS18B20 clones this driver was first tried on
  auto found = enumerate({0xDB0000065CDB4728ULL, 0x120000065C17EE28ULL});
  EXPECT_EQ(std::set<uint64_t>(found.begin(), found.end()),
            (std::set<uint64_t>{0x120000065C17EE28ULL, 0xDB0000065CDB4728ULL}));
  EXPECT_EQ(found.size(), 2u);
}

TEST(OneWireRmtSearch, FindsAllDevicesWithSharedPrefixes) {
  // Many devices with a common family code and neighbouring serial numbers, plus the extremes
  std::vector<uint64_t> devices = {0x28, 0x2128, 0x4128, 0x6128, 0x6129, 0xFFFFFFFFFFFFFFFFULL, 0x8000000000000001ULL};
  auto found = enumerate(devices);
  EXPECT_EQ(std::set<uint64_t>(found.begin(), found.end()), std::set<uint64_t>(devices.begin(), devices.end()));
  EXPECT_EQ(found.size(), devices.size());
}

TEST(OneWireRmtSearch, FindsEveryDeviceOfALargerBus) {
  std::vector<uint64_t> devices;
  uint64_t value = 0x9E3779B97F4A7C15ULL;  // pseudo random but fixed
  for (int i = 0; i < 40; i++) {
    value = value * 6364136223846793005ULL + 1442695040888963407ULL;
    devices.push_back(value | 1);
  }
  auto found = enumerate(devices);
  std::sort(devices.begin(), devices.end());
  devices.erase(std::unique(devices.begin(), devices.end()), devices.end());
  EXPECT_EQ(std::set<uint64_t>(found.begin(), found.end()), std::set<uint64_t>(devices.begin(), devices.end()));
  EXPECT_EQ(found.size(), devices.size());
}

TEST(OneWireRmtSearch, ABusFailureEndsTheSearchWithoutChangingTheState) {
  SimulatedBus bus({0x28, 0x2128});
  bus.fail_at_step_ = 10;
  uint8_t last_discrepancy = 0;
  bool last_device_flag = false;
  uint64_t address = 0;
  EXPECT_EQ(search_next(bus, last_discrepancy, last_device_flag, address), 0u);
  EXPECT_EQ(last_discrepancy, 0);
  EXPECT_FALSE(last_device_flag);
  EXPECT_EQ(address, 0u);
}

TEST(OneWireRmtSearch, AFailureWhileAnnouncingTheLastBitEndsTheSearch) {
  SimulatedBus bus({0x28});
  bus.fail_write_ = true;
  uint8_t last_discrepancy = 0;
  bool last_device_flag = false;
  uint64_t address = 0;
  EXPECT_EQ(search_next(bus, last_discrepancy, last_device_flag, address), 0u);
  EXPECT_FALSE(last_device_flag);
  EXPECT_EQ(address, 0u);
}

TEST(OneWireRmtSearch, DoesNotFindAnythingAfterTheLastDevice) {
  SimulatedBus bus({0x28});
  uint8_t last_discrepancy = 0;
  bool last_device_flag = false;
  uint64_t address = 0;
  EXPECT_EQ(search_next(bus, last_discrepancy, last_device_flag, address), 0x28u);
  EXPECT_TRUE(last_device_flag);
  bus.start();
  EXPECT_EQ(search_next(bus, last_discrepancy, last_device_flag, address), 0u);
}

}  // namespace esphome::gpio::testing
