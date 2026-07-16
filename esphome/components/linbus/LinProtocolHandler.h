#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_timer.h"  // ensure own direct dependency
#include "LinCommon.h"

#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Forward Declarations (to avoid circular dependencies)
class LinSerial;

// Constants for LIN protocol
#define LIN_BREAK 0x00
#define LIN_SYNC 0x55

// LIN frame structure
typedef struct {
  uint8_t id;                       // ID
  uint8_t dataLength;               // Length of data bytes
  uint8_t data[LIN_MAX_DATA_SIZE];  // Data bytes
  uint8_t checksum;                 // Checksum byte
} lin_frame_t;

// Structure to track discovered PIDs
typedef struct {
  uint8_t id;                       // ID
  uint8_t dataLength;               // Length of data observed
  uint8_t data[LIN_MAX_DATA_SIZE];  // Data bytes
  int64_t lastUpdateTime;           // Last time this PID was seen (microseconds)
  uint32_t updateCount;             // How many times this PID was observed
} lin_discovered_id_t;

typedef struct {
  uint8_t id;                  // ID
  uint32_t updateInterval;     // Send interval (ms); 0 marks a free slot
  uint32_t last_send_time_ms;  // millis() of last send (unsigned; wrap-safe subtraction)
  bool sent_once;              // false until first send (replaces the ambiguous 0 sentinel)
} lin_schedule_item_t;

typedef struct {
  uint8_t id;
  uint8_t dataLength;                            // Payload plus checksum; 0 marks a free slot
  uint8_t responseData[LIN_MAX_DATA_SIZE + 1];  // last byte = checksum
  bool enhancedChecksum;
  uint32_t lastResponseTime;
} lin_respond_data_t;
// NOTE: Free / unused response slot is now indicated by dataLength == 0 (ID may be any valid value, including 0).

// LIN Statistics structure (10-second rolling window)
typedef struct {
  // Windowed counters (reset every 10 s; feed the per-second rate fields below)
  uint32_t master_requests_sent;
  uint32_t id_requests_answered;
  uint32_t data_bytes_received;
  uint32_t frames_received;  // accepted (checksum-valid) frames this window (N6)
  uint32_t unique_ids_seen;

  // Windowed error counters — one per distinct fault class (N6 root-cause taxonomy)
  uint32_t master_send_failures;
  uint32_t pid_errors;
  uint32_t checksum_errors;
  uint32_t checksum_fallback;   // frame accepted with the *other* checksum mode (see UPGRADES.md C2)
  uint32_t frame_errors;        // UART framing/parity/OVF resyncs (spec "framing_errors", UPGRADES.md C3)
  uint32_t collisions;          // TX readback mismatch — competing publisher on a shared ID (N5)
  uint32_t frames_coalesced;    // on_frame drops when frames arrive faster than the loop drains (H3)
  uint32_t uart_breaks;         // UART_BREAK events actually delivered by the driver (HW-0b/HW-0c)

  // Lifetime totals (never reset -> safe for STATE_CLASS_TOTAL_INCREASING sensors, UPGRADES.md H4)
  uint32_t master_requests_total;
  uint32_t id_requests_answered_total;
  uint32_t data_bytes_received_total;
  uint32_t frames_received_total;
  uint32_t checksum_errors_total;
  uint32_t master_send_failures_total;
  uint32_t pid_errors_total;
  uint32_t frame_errors_total;
  uint32_t collisions_total;
  uint32_t checksum_fallback_total;
  uint32_t frames_coalesced_total;
  uint32_t uart_breaks_total;

  // Bus activity
  uint32_t total_bus_bytes;
  int64_t stats_start_time;

  // Per-second rates (calculated values)
  float master_reqs_per_sec;
  float id_answers_per_sec;
  float data_bytes_per_sec;
  float estimated_bus_load_percent;
  float error_rate_percent;
} lin_statistics_t;

enum lin_read_state_t {
  READ_STATE_BREAK,
  READ_STATE_SYNC,
  READ_STATE_SID,
  READ_STATE_DATA,
};

// Example: Handler for PID requests and responses
class LinProtocolHandler {
 public:
  // Pass schedule pointer to constructor
  LinProtocolHandler(LinSerial *serial, int baud_rate, lin_schedule_item_t *schedule,
                     lin_discovered_id_t *discoveredIDs, lin_respond_data_t *responses, lin_mode_t mode,
                     SemaphoreHandle_t data_mutex);
  virtual ~LinProtocolHandler();

  // Called when a PID is received, to check for registered responses
  void onID(uint8_t id);

  // Called when a data frame is received
  void onData(uint8_t pid, const uint8_t *data, size_t len);

  // Send the next due scheduled header (break + sync + PID). Called from the UART event
  // task on every wake (A5 off-loop tick), so the cadence is main-loop-independent (N8).
  // At most one header per call; returns the ID sent, or 0 if none was due / bus busy.
  uint8_t master_send();

  // Get data for a specific PID, returns length or 0 if not found
  size_t getDataForID(uint8_t id, uint8_t *dataBuf);

  // Statistics methods
  void updateStatistics();
  void resetStatistics();
  const lin_statistics_t &getStatistics() const;
  void logStatistics() const;

  // Called from the UART event task with each received byte (command = 0). Not an ISR —
  // but still the per-byte hot path: no logging, no heap, no blocking (N7).
  lin_read_state_t receiveDecode(uint8_t readbyte, uint8_t command);

  uint8_t calculateChecksum(uint8_t id, const uint8_t *data, size_t len, bool enhanced = true);

  // Per-received-ID static hints from config (set once at setup, before the event task runs).
  // A LIN frame's length and checksum type are static per ID (the LDF), so these let a
  // listener/sniffer frame and checksum IDs it never transmits. See spec A3 (length-aware
  // finalize) and C2/B6 (RX checksum control).
  void setRxExpectedLength(uint8_t id, uint8_t len);  // len 0..8; 0 = unknown (timeout fallback)
  void setRxChecksumMode(uint8_t id, bool enhanced);  // declare classic/enhanced for this ID

  void onInterByteTimeout();

  // Count frames dropped by main-loop-rate coalescing on the on_frame trigger (H3/N6). Called
  // from the control loop; folds into the frames_coalesced windowed + lifetime counters.
  void noteCoalesced(uint32_t n);

  // UART-event hooks (called from the UART event task, see UPGRADES.md C3)
  void onUartBreak();   // hardware BREAK: finalize any pending frame, resync to the sync byte
  void onFrameError();  // UART framing/parity error: reset the decoder and count it

 private:
  LinSerial *serial_;
  int baud_rate_;
  uint8_t uart_num_;
  lin_mode_t mode_;
  SemaphoreHandle_t data_mutex_;

  int64_t byte_time_us_;
  void resetDecodeState();

  lin_read_state_t state_;
  uint8_t currentID_;
  uint8_t currentPIDWithParity_;
  uint8_t currentData_[LIN_MAX_DATA_SIZE + 2];
  uint8_t currentDataCount_;

  // Per-ID RX hints (indexed by 6-bit ID). Written once at setup, then read lock-free in the
  // event task (never mutated after setup, so the unlocked read is safe).
  uint8_t rx_expected_len_[64] = {0};  // 0 = unknown; else payload length -> finalize at len+1
  int8_t rx_checksum_mode_[64];        // -1 unknown, 0 classic, 1 enhanced (init in ctor)
  // Expected total DATA-phase byte count (payload + checksum) for an ID, or -1 if unknown.
  int expectedFrameBytes_(uint8_t id) const;

  // Pointer to schedule array and count (owned by LINCommunication)
  lin_schedule_item_t *schedule_;

  // Rotating scan start for master_send() fairness: the off-loop tick calls it every ~2 ms,
  // so a pathologically short interval in a low slot would otherwise win the from-zero scan
  // every time and starve later slots. Consecutive calls resume after the last-sent slot.
  // Touched only under lockData().
  size_t schedule_scan_next_ = 0;

  // --- TX-readback collision detection (A6/N5/B9) ---
  // Every byte this node drives onto the one-wire bus echoes back into its own decoder. The
  // expected-echo FIFO holds what we just transmitted (ECHO_BREAK marks the break, whose echo
  // presentation varies — HW-0b vs HW-0c); receiveDecode() compares each incoming byte against
  // the front and counts a mismatch as a collision: a competing publisher overdrove us.
  // Single-context by design: since the A5 event-task fold, ALL TX (master_send headers, onID
  // responses) and ALL consumption (receiveDecode / onUartBreak / onInterByteTimeout /
  // onFrameError) run in the UART event task, so the FIFO itself needs no locking — only the
  // collision counter increments take the data mutex (like pid_errors). Static storage, no
  // heap on the hot path (N7); sized for a header (3) + a max response (9) with slack.
  static constexpr size_t ECHO_FIFO_SIZE = 16;
  static constexpr uint16_t ECHO_BREAK = 0x100;  // out-of-band break marker (> any data byte)
  uint16_t echo_fifo_[ECHO_FIFO_SIZE] = {};
  uint8_t echo_head_ = 0;
  uint8_t echo_count_ = 0;

  void echoCountCollision();  // collisions/collisions_total under lockData(0)
  void echoClear() {
    echo_head_ = 0;
    echo_count_ = 0;
  }
  void echoBeginTx();  // stale leftovers (previous echo never returned) -> one collision + clear
  void echoPush(uint16_t v);
  uint16_t echoFront() const { return echo_fifo_[echo_head_]; }
  void echoPop() {
    echo_head_ = (uint8_t) ((echo_head_ + 1) % ECHO_FIFO_SIZE);
    echo_count_--;
  }
  void echoCompare(uint8_t readbyte);  // consume/compare the front against an incoming byte

  lin_respond_data_t *responses_;

  lin_discovered_id_t *discoveredIDs_;

  // Statistics tracking
  lin_statistics_t statistics_;
  mutable lin_statistics_t statistics_snapshot_;
  uint8_t unique_id_tracker_[256];  // Bitmap to track unique IDs seen in current window

  bool lockData(TickType_t timeout = portMAX_DELAY) const;
  void unlockData() const;

  // Utility
  // Utility functions
  uint8_t calculate_pid(uint8_t id);

  uint8_t get_id_from_pid(uint8_t pid);

  int64_t last_bus_activity_us_ = 0;  // timestamp of last seen bus byte
 public:
  bool isBusIdle(uint32_t required_idle_bits = 3);
};
