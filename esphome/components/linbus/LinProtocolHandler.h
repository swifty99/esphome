#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_timer.h"  // ensure own direct dependency
#include "LinCommon.h"

#include "driver/uart.h"
#include "esp_intr_alloc.h"

// Forward Declarations (to avoid circular dependencies)
class LinSerial;

// LIN frame constants
#define LIN_BREAK_DURATION 13  // Break field duration (13 bits)
#define LIN_SYNC_BYTE 0x55     // Sync byte value

// Constants for LIN protocol
#define LIN_BREAK 0x00
#define LIN_SYNC 0x55
#define DIAGNOSTIC_FRAME_MASTER 0x3C
#define DIAGNOSTIC_FRAME_SLAVE 0x3D

// LIN frame structure
typedef struct {
  uint8_t id;                       //  ID
  uint8_t dataLength;               // Length of data (1-8 bytes)
  uint8_t data[LIN_MAX_DATA_SIZE];  // Data bytes
  uint8_t checksum;                 // Checksum byte
} lin_frame_t;

// Structure to track discovered PIDs
typedef struct {
  uint8_t id;                       //  ID
  uint8_t dataLength;               // Length of data observed
  uint8_t data[LIN_MAX_DATA_SIZE];  // Data bytes
  int64_t lastUpdateTime;           // Last time this PID was seen (microseconds)
  uint32_t updateCount;             // How many times this PID was observed
} lin_discovered_id_t;

typedef struct {
  uint8_t id;                 //  ID
  uint32_t updateInterval;    // Average interval between updates (ms)
  int32_t last_send_time_ms;  // Last time this PID was sent (microseconds)
} lin_schedule_item_t;

typedef struct {
  uint8_t id;
  uint8_t dataLength;
  uint8_t responseData[LIN_MAX_DATA_SIZE + 1];  // last byte = checksum
  uint32_t lastResponseTime;
} lin_respond_data_t;
// NOTE: Free / unused response slot is now indicated by dataLength == 0 (ID may be any valid value, including 0).

// LIN Statistics structure (10-second rolling window)
typedef struct {
  // Counters
  uint32_t master_requests_sent;
  uint32_t id_requests_answered;
  uint32_t data_bytes_received;
  uint32_t unique_ids_seen;

  // Error counters
  uint32_t master_send_failures;
  uint32_t pid_errors;
  uint32_t checksum_errors;

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
                     lin_discovered_id_t *discoveredIDs, lin_respond_data_t *responses);
  virtual ~LinProtocolHandler();

  // Called when a PID is received, to check for registered responses
  void onID(uint8_t id);

  // Called when a data frame is received
  void onData(uint8_t pid, const uint8_t *data, size_t len);

  // Called on protocol timeout
  void onTimeout();

  // put scheduled ID on bus
  uint8_t master_send();

  // Get data for a specific PID, returns length or 0 if not found
  size_t getDataForID(uint8_t id, uint8_t *dataBuf);

  // Dump all received PIDs to log
  void dumpReceivedPIDsToLog() const;

  // Statistics methods
  void updateStatistics();
  void resetStatistics();
  const lin_statistics_t &getStatistics() const { return statistics_; }
  void logStatistics() const;

  // Call by ISR handler ‚ to process incoming data
  lin_read_state_t receiveDecode(uint8_t readbyte, uint8_t command);

  uint8_t calculateChecksum(uint8_t id, const uint8_t *data, size_t len);

  void onInterByteTimeout();

 private:
  LinSerial *serial_;
  int baud_rate_;
  uint8_t uart_num_;

  int64_t byte_time_us_;
  int64_t timeout_LinRecieive_;
  void resetDecodeState();

  // --- Circular buffer for received PIDs/data ---
  struct ReceivedEntry {
    uint8_t pid;
    uint8_t data[16];
    size_t len;
  };

  lin_read_state_t state_;
  uint8_t currentID_;
  uint8_t currentPIDWithParity_;
  uint8_t currentData_[LIN_MAX_DATA_SIZE + 2];
  uint8_t currentDataCount_;
  int64_t lastDataReceived;

  // Pointer to schedule array and count (owned by LINCommunication)
  lin_schedule_item_t *schedule_;

  lin_respond_data_t *responses_;

  lin_discovered_id_t *discoveredIDs_;

  // Statistics tracking
  lin_statistics_t statistics_;
  uint8_t unique_id_tracker_[256];  // Bitmap to track unique IDs seen in current window

  // Utility
  // Utility functions
  uint8_t calculate_pid(uint8_t id);

  uint8_t get_id_from_pid(uint8_t pid);

  int64_t last_bus_activity_us_ = 0;  // timestamp of last seen bus byte
 public:
  bool isBusIdle(uint32_t required_idle_bits = 3);
};
