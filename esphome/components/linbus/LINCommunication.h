#ifndef LIN_COMMUNICATION_H
#define LIN_COMMUNICATION_H

// Standard Library Headers
#include <cstdint>
#include <cstddef>

// Third-Party Library Headers
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "LinCommon.h"

// Forward Declarations
class LinProtocolHandler;
class LinSerial;

/**
 * @file LINCommunication.h
 * @brief Local Interconnect Network (LIN) communication driver
 *
 * This library implements a non-blocking LIN communication driver using a layered,
 * event-driven architecture for improved maintainability and low latency.
 *
 * The LINCommunication class integrates three main components:
 * - LinSerial: Handles UART serial I/O
 * - LinStateMachine: Implements the LIN protocol state machine, parsing frames and emitting events
 * - LinProtocolHandler: Handles PID requests, decodes frames, and sends responses
 *
 * # Operation Modes
 *
 * ## Master Mode
 * - Sends scheduled headers from the UART event task (A5 off-loop tick) — cadence is
 *   main-loop-independent (N2/N8)
 * - Can respond to its own PIDs when response data is set
 *
 * ## Slave Mode
 * - Listens for PIDs on the bus
 * - Responds automatically when a matching PID is detected and response data has been set
 *
 * ## Listener Mode
 * - Passively monitors LIN bus traffic without responding
 * - Tracks discovered PIDs and their properties
 *
 * # Usage
 *
 * 1. Create a LINCommunication object with UART port and pins
 * 2. Initialize with desired mode and baud rate
 * 3. For master: Add IDs to the schedule with master_add_id_schedule() — headers then go
 *    out from the event task on their own cadence
 * 4. For slave/responder: Set response data with set_response_data()
 * 5. Call lin_process() regularly in your main loop for the loop-context housekeeping
 *    (self-test evaluation, statistics windows) — the data path does not depend on it
 *
 * This design separates concerns into distinct layers, making the code easier to maintain,
 * extend, and test. The protocol logic is event-driven and non-blocking, suitable for embedded real-time systems.
 */

// Core constants (define only if not already defined elsewhere)

#include "LinProtocolHandler.h"

class LINCommunication {
 public:
  LINCommunication(uart_port_t uart_num, int tx_pin, int rx_pin, int cs_pin = -1);
  esp_err_t init(lin_mode_t mode, int baud_rate);  // Initialize LIN communication
  ~LINCommunication();

  bool lin_process();  // Loop-context housekeeping: self-test evaluation + statistics windows.
                       // The data path (RX decode, response TX, master schedule) runs in the
                       // UART event task and does not depend on this being called.

  // Master scheduler
  bool master_add_id_schedule(uint8_t id, uint32_t interval_ms);
  bool master_remove_id_schedule(uint8_t id);

  // Responses
  bool set_response_data(uint8_t id, uint8_t *data, uint8_t data_length, bool enhanced = true,
                         bool preserve_existing_checksum = false);
  bool get_response_4id(uint8_t id, lin_discovered_id_t *lindata);

  // Per-received-ID static hints from config (length-aware finalize + declared checksum mode).
  // length 0 = no length hint; checksum_mode -1 = undeclared, 0 = classic, 1 = enhanced.
  // Call after init() (the protocol handler must exist). See spec A3/C2.
  void configure_rx_hint(uint8_t id, uint8_t length, int8_t checksum_mode);

  // Fold main-loop-rate on_frame coalescing into the counters (H3/N6).
  void note_frames_coalesced(uint32_t n);

  // Built-in self test: whether every exercised ID has reached the pass threshold (S6/B15).
  bool self_test_passed() const { return self_test_passed_; }

  // Debug
  void dumpReceivedPIDsToLog() const;

  // Enable/disable built-in LIN self test (master only). Call before or after init().
  void enableSelfTest(bool enable);

  // Statistics (low overhead access to LinProtocolHandler statistics)
  const lin_statistics_t &getStatistics() const;
  void logStatistics() const;
  void resetStatistics();
  void updateStatistics();

 private:
  uart_port_t uart_num_;
  int tx_pin_;
  int rx_pin_;
  int cs_pin_;
  lin_mode_t mode_;
  int baud_rate_;

  LinSerial *lin_serial_{nullptr};
  LinProtocolHandler *lin_protocol_handler_{nullptr};
  bool is_initialized_ = false;
  mutable SemaphoreHandle_t data_mutex_{nullptr};

  lin_schedule_item_t schedule_[LIN_MAX_ID_RQ_COUNT];
  lin_respond_data_t responses_[LIN_MAX_RESPONSES];
  lin_discovered_id_t discoveredIDs_[LIN_MAX_DISCOVERED_PIDS];

  TaskHandle_t uart_event_task_handle_{nullptr};
  static void uart_event_task_trampoline(void *arg);

  // Inter-byte timeout in microseconds (≈ 3 byte times at current baud)
  uint32_t inter_byte_timeout_us_ = 0;

  // test stuff
  bool self_test_enabled_ = false;
  bool self_test_initialized_ = false;
  bool self_test_passed_ = false;
  int64_t self_test_start_time_us_ = 0;
  int64_t self_test_last_eval_us_ = 0;  // per-instance (was a function-local static, UPGRADES.md M6)

  struct SelfTestStat {
    uint8_t id;
    uint32_t scheduled_interval_ms;
    uint32_t seen_updates;
    int64_t last_update_time_us;
    int64_t first_update_time_us;
    uint32_t missed_windows;  // intervals where no update arrived
    int64_t prev_update_time_us;
    int64_t min_interval_us;
    int64_t max_interval_us;
  };
  static constexpr size_t SELF_TEST_MAX_IDS = 10;
  SelfTestStat self_test_stats_[SELF_TEST_MAX_IDS]{};

  void selfTestTick();  // invoked inside lin_process()
  void selfTestInitIfNeeded();
  void selfTestEvaluate();
};

#endif  // LIN_COMMUNICATION_H
