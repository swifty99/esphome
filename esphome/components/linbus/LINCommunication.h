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
 * - Intended to schedule and transmit PIDs on the bus (scheduling logic to be implemented)
 * - Can respond to its own PIDs when response data is set
 *
 * ## Slave Mode
 * - Listens for PIDs on the bus
 * - Responds automatically when a matching PID is detected and response data has been set
 *
 * ## Listener Mode
 * - Passively monitors LIN bus traffic without responding
 * - Tracks discovered PIDs and their properties (to be implemented)
 *
 * # Usage
 *
 * 1. Create a LINCommunication object with UART port and pins
 * 2. Initialize with desired mode and baud rate
 * 3. For master: Add PIDs to schedule with master_add_pid_schedule() (not yet implemented)
 * 4. For slave/responder: Set response data with set_response_data()
 * 5. Call lin_process() regularly in your main loop to process LIN frames
 *
 * This design separates concerns into distinct layers, making the code easier to maintain,
 * extend, and test. The protocol logic is event-driven and non-blocking, suitable for embedded real-time systems.
 */

// Core constants (define only if not already defined elsewhere)

// LIN operating modes
typedef enum { LIN_MODE_MASTER, LIN_MODE_SLAVE, LIN_MODE_LISTENER } lin_mode_t;

#include "LinProtocolHandler.h"

class LINCommunication {
 public:
  LINCommunication(uart_port_t uart_num, int tx_pin, int rx_pin, int cs_pin = -1);
  esp_err_t init(lin_mode_t mode, int baud_rate);  // Initialize LIN communication
  ~LINCommunication();

  bool lin_process();  // Process LIN communication in loop, slave_send_response() and calls master_sheduler() if in
                       // master mode

  // Master scheduler
  bool master_add_id_schedule(uint8_t id, uint32_t interval_ms);
  bool master_remove_id_schedule(uint8_t id);

  // Responses
  bool set_response_data(uint8_t id, uint8_t *data, uint8_t data_length);
  bool get_response_4id(uint8_t id, lin_discovered_id_t *lindata);

  // Debug
  void dumpReceivedPIDsToLog()
      const;  // Enable/disable built‑in LIN self test (master only). Call before or after init().
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

  LinSerial *lin_serial_;
  LinProtocolHandler *lin_protocol_handler_;
  bool is_initialized_ = false;

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
  int64_t self_test_start_time_us_ = 0;

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

/**
 * @brief Define LIN_DEBUG_LOGGING to enable ESP_LOG* debug output in LIN communication.
 * Undefine or comment out to disable all LIN debug logging for real-time testing.
 */
#define LIN_DEBUG_LOGGING

#endif  // LIN_COMMUNICATION_H
