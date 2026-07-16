#include "LinProtocolHandler.h"
// Third-Party Library Headers
#include "LinSerial.h"  // Explicit include for LinSerial
#include "LinCommon.h"  // Explicit include for shared constants
#include "esp_log.h"    // Explicit include for ESP_LOG* macros
#include "esp_timer.h"  // Explicit include for esp_timer_get_time()
#include <cstring>      // For memset

#include "driver/uart.h"

#include <cstring>

static const char *TAG = "LinProt";

LinProtocolHandler::LinProtocolHandler(LinSerial *serial, int baud_rate, lin_schedule_item_t *schedule,
                                       lin_discovered_id_t *discoveredIDs, lin_respond_data_t *responses,
                                       lin_mode_t mode, SemaphoreHandle_t data_mutex)
    : serial_(serial),
      baud_rate_(baud_rate),
      mode_(mode),
      data_mutex_(data_mutex),
      state_(READ_STATE_BREAK),
      schedule_(schedule),
      responses_(responses),
      discoveredIDs_(discoveredIDs) {
  byte_time_us_ = 1000000 / baud_rate_ * 10;  // 10 bits per byte at 19200 baud ≈ 520us

  // Per-ID RX hints: no length known and no checksum mode declared until config sets them.
  memset(rx_expected_len_, 0, sizeof(rx_expected_len_));
  memset(rx_checksum_mode_, -1, sizeof(rx_checksum_mode_));  // 0xFF bytes -> int8_t -1

  // Initialize statistics
  resetStatistics();
}

// Destructor to free buffer
LinProtocolHandler::~LinProtocolHandler() { ; }

bool LinProtocolHandler::lockData(TickType_t timeout) const {
  return data_mutex_ == nullptr || xSemaphoreTake(data_mutex_, timeout) == pdTRUE;
}

void LinProtocolHandler::unlockData() const {
  if (data_mutex_ != nullptr) {
    xSemaphoreGive(data_mutex_);
  }
}

void LinProtocolHandler::onID(uint8_t id) {
  if (!serial_ || mode_ == LIN_MODE_LISTENER)
    return;
  if (!lockData())
    return;
  for (size_t i = 0; i < LIN_MAX_RESPONSES; ++i) {
    if (responses_[i].id == id && responses_[i].dataLength) {
      serial_->writeBytes(responses_[i].responseData, responses_[i].dataLength);

      // Update statistics - ID request answered
      statistics_.id_requests_answered++;
      statistics_.id_requests_answered_total++;
      statistics_.total_bus_bytes += responses_[i].dataLength;

      // Copy the just-written bytes while still under the mutex (the main loop may mutate
      // responses_ after we release), then register the echo expectation OUTSIDE the lock:
      // echoBeginTx()/echoPush() can count a collision, which takes lockData(0) itself —
      // the mutex is non-recursive, so pushing while locked would silently drop the count.
      uint8_t echo_len = responses_[i].dataLength;
      uint8_t echo_bytes[LIN_MAX_DATA_SIZE + 1];
      memcpy(echo_bytes, responses_[i].responseData, echo_len);
      unlockData();

      echoBeginTx();
      for (uint8_t k = 0; k < echo_len; ++k)
        echoPush(echo_bytes[k]);
      return;
    }
  }
  unlockData();
}

void LinProtocolHandler::onData(uint8_t pid, const uint8_t *data, size_t len) {
  // Per-frame hot path (event task): no logging here (N7); counters carry the observability.
  if (len == 0 || len > 16)
    return;

  if (!lockData())
    return;

  // Update statistics - data received
  statistics_.data_bytes_received += len;
  statistics_.data_bytes_received_total += len;
  statistics_.total_bus_bytes += len;

  // Track unique IDs
  uint8_t id = pid & 0x3F;
  if (!(unique_id_tracker_[id / 8] & (1 << (id % 8)))) {
    unique_id_tracker_[id / 8] |= (1 << (id % 8));
    statistics_.unique_ids_seen++;
  }

  // Preferred checksum mode: the checksum type is a *static per-ID property* (defined in the
  // LDF), not a function of who transmits it. Prefer an explicitly declared RX mode (rx_ids /
  // on_frame checksum, spec C2/B6); otherwise infer it from the response this node publishes
  // for the ID; otherwise default enhanced. If the preferred mode fails we retry the other
  // mode instead of dropping the frame, so a listener/sniffer still receives classic (LIN 1.x)
  // traffic it never transmits — counted as checksum_fallback. See UPGRADES.md C2.
  bool enhanced_checksum = true;
  int8_t declared_mode = rx_checksum_mode_[id & 0x3F];
  if (declared_mode >= 0) {
    enhanced_checksum = (declared_mode != 0);
  } else {
    for (size_t i = 0; i < LIN_MAX_RESPONSES; ++i) {
      if (responses_[i].id == id && responses_[i].dataLength) {
        enhanced_checksum = responses_[i].enhancedChecksum;
        break;
      }
    }
  }

  uint8_t expectedChecksum = calculateChecksum(pid, data, len - 1, enhanced_checksum);
  if (data[len - 1] != expectedChecksum) {
    uint8_t altChecksum = calculateChecksum(pid, data, len - 1, !enhanced_checksum);
    if (data[len - 1] == altChecksum) {
      // Accept, but record that we needed the other checksum mode.
      statistics_.checksum_fallback++;
      statistics_.checksum_fallback_total++;
    } else {
      ESP_LOGD(TAG, "Checksum error for PID 0x%02X: got 0x%02X, expected 0x%02X", pid, data[len - 1],
               expectedChecksum);
      statistics_.checksum_errors++;
      statistics_.checksum_errors_total++;
      unlockData();
      return;
    }
  }

  // Frame accepted (checksum valid under the preferred or fallback mode).
  statistics_.frames_received++;
  statistics_.frames_received_total++;

  // id already declared above - reuse it
  int oldest_idx = -1;
  int64_t oldest_time = INT64_MAX;

  // Scan for existing entry (match by ID) or first free (dataLength == 0)
  for (size_t i = 0; i < LIN_MAX_DISCOVERED_PIDS; ++i) {
    if (discoveredIDs_[i].dataLength > 0) {
      // Used slot
      if (discoveredIDs_[i].id == id) {
        size_t copyLen = (len - 1 > LIN_MAX_DATA_SIZE) ? LIN_MAX_DATA_SIZE : (len - 1);
        memcpy(discoveredIDs_[i].data, data, copyLen);
        discoveredIDs_[i].dataLength = copyLen;
        discoveredIDs_[i].updateCount++;
        discoveredIDs_[i].lastUpdateTime = esp_timer_get_time();
        unlockData();
        return;
      }
      if (discoveredIDs_[i].lastUpdateTime < oldest_time) {
        oldest_time = discoveredIDs_[i].lastUpdateTime;
        oldest_idx = (int) i;
      }
    } else {
      // Free slot (dataLength == 0). Use it.
      discoveredIDs_[i].id = id;
      size_t copyLen = (len - 1 > LIN_MAX_DATA_SIZE) ? LIN_MAX_DATA_SIZE : (len - 1);
      memcpy(discoveredIDs_[i].data, data, copyLen);
      discoveredIDs_[i].dataLength = copyLen;
      discoveredIDs_[i].updateCount = 1;
      discoveredIDs_[i].lastUpdateTime = esp_timer_get_time();
      ESP_LOGI(TAG, "Discovered new ID 0x%02X", id);
      unlockData();
      return;
    }
  }

  // All slots used – overwrite oldest
  if (oldest_idx >= 0) {
    size_t copyLen = (len - 1 > LIN_MAX_DATA_SIZE) ? LIN_MAX_DATA_SIZE : (len - 1);
    discoveredIDs_[oldest_idx].id = id;
    memcpy(discoveredIDs_[oldest_idx].data, data, copyLen);
    discoveredIDs_[oldest_idx].dataLength = copyLen;
    discoveredIDs_[oldest_idx].updateCount = 1;
    discoveredIDs_[oldest_idx].lastUpdateTime = esp_timer_get_time();
    ESP_LOGD(TAG, "Overwrote oldest slot with ID 0x%02X", id);
  }
  unlockData();
}

// Runs in the UART event task (A5 off-loop tick) — the same context as the response TX in
// onID(), so header and response bytes can never interleave on the wire. lockData(0) is
// non-blocking: if the main loop holds the mutex (schedule/response mutation) this wake is
// skipped and the next one (≤ ~2 ms later) retries; due-ness is not consumed by a skip.
uint8_t LinProtocolHandler::master_send() {
  if (mode_ != LIN_MODE_MASTER)
    return 0;
  if (!lockData(0))
    return 0;
  // Unsigned millis with wrap-safe subtraction: no int32 overflow at ~24.8 days. See UPGRADES.md C4.
  uint32_t current_time_ms = (uint32_t) (esp_timer_get_time() / 1000);
  for (size_t k = 0; k < LIN_MAX_ID_RQ_COUNT; k++) {
    // Rotating scan start: resume after the last-sent slot so an always-due short interval
    // in a low slot cannot starve later slots at the ~2 ms tick rate.
    size_t i = (schedule_scan_next_ + k) % LIN_MAX_ID_RQ_COUNT;
    if (schedule_[i].updateInterval == 0)
      continue;  // free slot sentinel

    uint32_t elapsed_ms = current_time_ms - schedule_[i].last_send_time_ms;
    if (!schedule_[i].sent_once || elapsed_ms >= schedule_[i].updateInterval) {
      if (!isBusIdle(3)) {
        unlockData();
        return 0;
      }

      uint8_t id = schedule_[i].id;
      bool sent = false;
      if (serial_) {
        sent = serial_->sendBreak();
        if (sent) {
          uint8_t sync = 0x55;
          sent = serial_->writeBytes(&sync, 1);
          uint8_t pid = calculate_pid(id);
          sent = serial_->writeBytes(&pid, 1) && sent;
        }
      }
      if (sent) {
        // Update statistics - master request sent
        statistics_.master_requests_sent++;
        statistics_.master_requests_total++;
        statistics_.total_bus_bytes += 3;  // break + sync + pid
      } else {
        // Update statistics - real master send failure (TX error/timeout), see UPGRADES.md M2
        statistics_.master_send_failures++;
        statistics_.master_send_failures_total++;
      }
      schedule_[i].last_send_time_ms = current_time_ms;
      schedule_[i].sent_once = true;
      schedule_scan_next_ = (i + 1) % LIN_MAX_ID_RQ_COUNT;
      unlockData();
      // Register the header's echo expectation (A6) — outside the lock, see onID(). Only on a
      // successful send: a failed TX is already counted as send_failure, and whatever fragment
      // reached the wire passes as foreign traffic rather than faking a collision.
      if (sent) {
        echoBeginTx();
        echoPush(ECHO_BREAK);
        echoPush(0x55);
        echoPush(calculate_pid(id));
      }
      return id;
    }
  }
  unlockData();
  return 0;
}

// FIX: Treat argument as PID (with parity) OR raw ID. Always mask to 6-bit ID
// and only include PID in checksum for enhanced frames (non-diagnostic).
uint8_t LinProtocolHandler::calculateChecksum(uint8_t pid_or_id, const uint8_t *data, size_t len, bool enhanced) {
  uint8_t id = pid_or_id & 0x3F;  // 6-bit ID
  uint16_t sum = 0;

  // Enhanced checksum (LIN 2.x): include protected identifier (with parity)
  // Classic checksum: data bytes only.
  // Diagnostic frames (0x3C, 0x3D) ALWAYS use classic checksum.
  if (enhanced && id != 0x3C && id != 0x3D) {
    // Use canonical PID value (rebuild from ID) to avoid parity inconsistencies
    uint8_t pid = calculate_pid(id);
    sum += pid;
  }

  for (size_t i = 0; i < len; ++i) {
    sum += data[i];
  }

  // Fold carries
  while (sum > 0xFF) {
    sum = (sum & 0xFF) + (sum >> 8);
  }
  return (uint8_t) (~sum);
}

// Get data for a specific PID, returns length or 0 if not found
size_t LinProtocolHandler::getDataForID(uint8_t id, uint8_t *dataBuf) {
  if (!lockData())
    return 0;
  for (size_t i = 0; i < LIN_MAX_DISCOVERED_PIDS; ++i) {
    if (discoveredIDs_[i].dataLength == 0)
      continue;  // skip free
    if (discoveredIDs_[i].id == id) {
      memcpy(dataBuf, discoveredIDs_[i].data, discoveredIDs_[i].dataLength);
      size_t data_length = discoveredIDs_[i].dataLength;
      unlockData();
      return data_length;
    }
  }
  unlockData();
  return 0;
}

void inline LinProtocolHandler::resetDecodeState() { state_ = READ_STATE_BREAK; }

void LinProtocolHandler::setRxExpectedLength(uint8_t id, uint8_t len) {
  if ((id & 0x3F) != id)
    return;
  if (len > LIN_MAX_DATA_SIZE)
    len = LIN_MAX_DATA_SIZE;
  rx_expected_len_[id] = len;
}

void LinProtocolHandler::setRxChecksumMode(uint8_t id, bool enhanced) {
  if ((id & 0x3F) != id)
    return;
  rx_checksum_mode_[id] = enhanced ? 1 : 0;
}

int LinProtocolHandler::expectedFrameBytes_(uint8_t id) const {
  uint8_t len = rx_expected_len_[id & 0x3F];
  if (len == 0)
    return -1;                // unknown -> caller uses the inter-byte-timeout fallback
  return static_cast<int>(len) + 1;  // payload + checksum byte
}

void LinProtocolHandler::noteCoalesced(uint32_t n) {
  if (n == 0)
    return;
  if (!lockData(0))
    return;
  statistics_.frames_coalesced += n;
  statistics_.frames_coalesced_total += n;
  unlockData();
}

// --- TX-readback collision detection (A6/N5/B9) — see the header for the context/locking
// contract. The FIFO is only ever touched from the UART event task. ---

void LinProtocolHandler::echoCountCollision() {
  if (lockData(0)) {
    statistics_.collisions++;
    statistics_.collisions_total++;
    unlockData();
  }
}

void LinProtocolHandler::echoBeginTx() {
  // Leftover expectations at the start of a new TX mean the previous echo never fully
  // returned although the write succeeded — count ONE collision and start clean (N6: one
  // cause, one counter; the inter-byte timeout usually catches this first).
  if (echo_count_ > 0) {
    echoCountCollision();
    echoClear();
  }
}

void LinProtocolHandler::echoPush(uint16_t v) {
  if (echo_count_ >= ECHO_FIFO_SIZE) {
    // Cannot happen (header 3 + max response 9 ≤ 16) — defensive guard, never wedge (N3).
    echoCountCollision();
    echoClear();
  }
  echo_fifo_[(echo_head_ + echo_count_) % ECHO_FIFO_SIZE] = v;
  echo_count_++;
}

void LinProtocolHandler::echoCompare(uint8_t readbyte) {
  uint16_t front = echoFront();
  if (front == ECHO_BREAK) {
    // Our own break echoes as a bare 0x00 data byte on this silicon (HW-0b). Anything else
    // while we drove the bus dominant means another node overdrove us.
    echoPop();
    if (readbyte != 0x00)
      echoCountCollision();
    return;
  }
  if (front == 0x55 && readbyte == 0x00) {
    // A UART_BREAK event may already have consumed the break marker (HW-0c) while the
    // break's 0x00 data byte still trails in. Tolerate without popping — mirrors the
    // decoder's stray-0x00-in-SYNC tolerance (B3).
    return;
  }
  echoPop();
  if (readbyte != (uint8_t) front)
    echoCountCollision();
}

void LinProtocolHandler::onInterByteTimeout() {
  if (state_ == READ_STATE_DATA) {
    // If we already have at least 2 bytes (payload+checksum min) finalize
    if (currentDataCount_ >= 2) {
      onData(currentPIDWithParity_, currentData_, currentDataCount_);
    }
    resetDecodeState();
  }
  // A6 desync recovery: expectations still queued after an idle gap mean our echo never
  // arrived (bus held dominant / overdriven end-to-end) -> one collision, start clean.
  if (echo_count_ > 0) {
    echoCountCollision();
    echoClear();
  }
}

void LinProtocolHandler::onUartBreak() {
  // A6: a genuine external-style break can surface as the UART_BREAK event instead of a
  // 0x00 data byte (HW-0c) — if we are expecting our own break's echo, this event is it.
  if (echo_count_ > 0 && echoFront() == ECHO_BREAK)
    echoPop();
  // A hardware BREAK deterministically ends the previous frame slot and starts a new
  // header. Finalize any complete pending frame, then wait for the sync byte. This is the
  // authoritative resync and removes the "ghost header" collision risk. See UPGRADES.md C3.
  if (state_ == READ_STATE_DATA && currentDataCount_ >= 2) {
    onData(currentPIDWithParity_, currentData_, currentDataCount_);
  }
  currentDataCount_ = 0;
  state_ = READ_STATE_SYNC;  // expect 0x55 next; a stray 0x00 break byte is tolerated in SYNC
  last_bus_activity_us_ = esp_timer_get_time();
  // Count that the driver actually delivered a UART_BREAK event. On the C6 self-loopback this
  // stays 0 (the break arrives as a 0x00 data byte, HW-0b); a climbing count on a two-node bus
  // answers HW-0c — a genuine external break does raise the event. Locked separately, after the
  // onData() above has already released the mutex (no re-entrancy).
  if (lockData(0)) {
    statistics_.uart_breaks++;
    statistics_.uart_breaks_total++;
    unlockData();
  }
}

void LinProtocolHandler::onFrameError() {
  // A6: corrupted wire state — drop any echo expectations silently; frame_errors (below)
  // already counts this fault, and double-counting it as a collision would blur the N6
  // taxonomy.
  echoClear();
  if (lockData(0)) {
    statistics_.frame_errors++;
    statistics_.frame_errors_total++;
    unlockData();
  }
  // Event-order robustness (N3/B3): a UART_FRAME_ERR can *trail* the very break that
  // onUartBreak() just resynced on (a real 13-bit break violates framing). If we are already
  // waiting for the sync byte, keep waiting — do not bounce back to BREAK and swallow the
  // following 0x55. In any other state, drop the in-flight frame and resync from scratch;
  // a corrupt byte must never be finalized into a frame.
  if (state_ != READ_STATE_SYNC) {
    resetDecodeState();  // -> BREAK; partial data is discarded on the next break byte
  }
}

lin_read_state_t LinProtocolHandler::receiveDecode(uint8_t readbyte, uint8_t command) {
  // Called from the UART event task with each received byte (command = 0). Not an ISR, but
  // still the per-byte hot path: straight fall-through state machine, no logging, no heap,
  // no blocking, so response TX timing (onID) stays tight (N7).

  if (command == 0) {
    last_bus_activity_us_ = esp_timer_get_time();
    // TX readback (A6/N5): if we are expecting our own echo, compare-and-consume first. The
    // byte then ALWAYS continues through the state machine below — a mismatch counts a
    // collision but never corrupts decode state (B9). Empty FIFO (foreign traffic, the
    // steady listener/slave-idle path) costs one comparison.
    if (echo_count_ > 0)
      echoCompare(readbyte);
  }

  switch (state_) {
    case READ_STATE_BREAK:
      currentDataCount_ = 0;   // init for later data receive
      if (readbyte == 0x00) {  // LIN_BREAK
        state_ = READ_STATE_SYNC;
      }
      break;
    case READ_STATE_SYNC:
      if (readbyte == 0x55) {  // LIN_SYNC
        state_ = READ_STATE_SID;
      } else if (readbyte == 0x00) {
        // Stray break byte before sync (e.g. after onUartBreak() resync): keep waiting for 0x55.
      } else {
        state_ = READ_STATE_BREAK;
      }
      break;
    case READ_STATE_SID: {
      // Accept any PID (0..63 with parity)
      uint8_t pid = readbyte;
      uint8_t id = pid & 0x3F;
      if (calculate_pid(id) == pid) {
        currentPIDWithParity_ = pid;
        currentID_ = id;

        // Immediately transmit response (first data byte timing critical)
        onID(id);

        // Enter data phase to capture response (including our own echo HW loops back)
        currentDataCount_ = 0;
        state_ = READ_STATE_DATA;
      } else {
        // Parity error -> reset
        if (lockData(0)) {
          statistics_.pid_errors++;
          statistics_.pid_errors_total++;
          unlockData();
        }
        state_ = READ_STATE_BREAK;
      }
      break;
    }

    case READ_STATE_DATA:
      if (command == 0) {  // only real bytes
        if (currentDataCount_ < (LIN_MAX_DATA_SIZE + 1)) {
          currentData_[currentDataCount_++] = readbyte;
          // Length-aware finalize (A3/H1): when the ID's frame length is known, end the frame
          // deterministically after exactly length+1 bytes (payload + checksum) instead of
          // waiting for the inter-byte silence. This makes back-to-back frames and
          // within-budget mid-response pauses safe, and is the primary end-of-frame boundary
          // on silicon that never signals a hardware BREAK (HW-0b). Silence stays the fallback.
          int expect = expectedFrameBytes_(currentID_);
          if (expect > 0 && (int) currentDataCount_ >= expect) {
            onData(currentPIDWithParity_, currentData_, currentDataCount_);
            resetDecodeState();
          }
        } else {
          // Overflow guard (unknown length): finalize what we have and resync.
          if (currentDataCount_ >= 2) {
            onData(currentPIDWithParity_, currentData_, currentDataCount_);
          }
          resetDecodeState();
        }
      }
      break;

    default:
      resetDecodeState();
      break;
  }
  return state_;
}

// Helper: check bus idle (driver-only)
bool LinProtocolHandler::isBusIdle(uint32_t required_idle_bits) {
  if (required_idle_bits == 0)
    required_idle_bits = 3;
  int64_t now = esp_timer_get_time();
  int64_t bit_time_us = (baud_rate_ > 0) ? (1000000LL / baud_rate_) : 52;
  int64_t needed_us = (int64_t) required_idle_bits * bit_time_us;

  if (state_ == READ_STATE_DATA)
    return false;
  if ((now - last_bus_activity_us_) < needed_us)
    return false;

  if (serial_ == nullptr)
    return false;
  if (uart_wait_tx_done(serial_->getPort(), 0) != ESP_OK)
    return false;

  return true;
}

// Utility functions remain unchanged
uint8_t LinProtocolHandler::calculate_pid(uint8_t id) {
  uint8_t p0 = ((id >> 0) ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 0x01;
  uint8_t p1 = ~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5)) & 0x01;
  uint8_t pid = id | (p0 << 6) | (p1 << 7);
  return pid;
}

uint8_t LinProtocolHandler::get_id_from_pid(uint8_t pid) {
  uint8_t id = pid & 0x3F;
  if (calculate_pid(id) == pid) {
    return id;
  } else {
    return 0x3F;  // Invalid PID
  }
}

// Statistics implementation
void LinProtocolHandler::resetStatistics() {
  if (!lockData())
    return;
  memset(&statistics_, 0, sizeof(statistics_));
  memset(&statistics_snapshot_, 0, sizeof(statistics_snapshot_));
  memset(unique_id_tracker_, 0, sizeof(unique_id_tracker_));
  statistics_.stats_start_time = esp_timer_get_time();
  unlockData();
}

void LinProtocolHandler::updateStatistics() {
  if (!lockData())
    return;
  int64_t now = esp_timer_get_time();
  int64_t window_duration_us = now - statistics_.stats_start_time;

  // Update every 10 seconds (10,000,000 microseconds)
  if (window_duration_us >= 10000000) {
    // Debug: Show raw counters before calculation
    ESP_LOGD(TAG, "Stats window: master_reqs=%u, id_answers=%u, data_bytes=%u, errors=%u+%u+%u",
             statistics_.master_requests_sent, statistics_.id_requests_answered, statistics_.data_bytes_received,
             statistics_.master_send_failures, statistics_.pid_errors, statistics_.checksum_errors);

    // Calculate rates (per second)
    double window_duration_s = window_duration_us / 1000000.0;

    statistics_.master_reqs_per_sec = statistics_.master_requests_sent / window_duration_s;
    statistics_.id_answers_per_sec = statistics_.id_requests_answered / window_duration_s;
    statistics_.data_bytes_per_sec = statistics_.data_bytes_received / window_duration_s;

    // Calculate bus load percentage
    // Assuming 8N1 format: 1 start + 8 data + 1 stop = 10 bits per byte
    int64_t theoretical_max_bits = (int64_t) baud_rate_ * window_duration_s;
    int64_t actual_bits = statistics_.total_bus_bytes * 10;
    statistics_.estimated_bus_load_percent =
        (theoretical_max_bits > 0) ? (double) actual_bits / theoretical_max_bits * 100.0 : 0.0;

    // Calculate error rates
    int64_t total_transactions = statistics_.master_requests_sent + statistics_.id_requests_answered;
    int64_t total_errors = statistics_.master_send_failures + statistics_.pid_errors + statistics_.checksum_errors;
    statistics_.error_rate_percent =
        (total_transactions > 0) ? (double) total_errors / total_transactions * 100.0 : 0.0;

    // Reset for next window (windowed counters only; lifetime *_total fields never reset)
    statistics_.stats_start_time = now;
    statistics_.master_requests_sent = 0;
    statistics_.id_requests_answered = 0;
    statistics_.data_bytes_received = 0;
    statistics_.frames_received = 0;
    statistics_.master_send_failures = 0;
    statistics_.pid_errors = 0;
    statistics_.checksum_errors = 0;
    statistics_.checksum_fallback = 0;
    statistics_.frame_errors = 0;
    statistics_.collisions = 0;
    statistics_.frames_coalesced = 0;
    statistics_.uart_breaks = 0;
    statistics_.total_bus_bytes = 0;
  }
  unlockData();
}

const lin_statistics_t &LinProtocolHandler::getStatistics() const {
  if (lockData()) {
    statistics_snapshot_ = statistics_;
    unlockData();
  }
  return statistics_snapshot_;
}

void LinProtocolHandler::logStatistics() const {
  const lin_statistics_t &stats = getStatistics();
  ESP_LOGI(TAG, "LIN Statistics (10s window):");
  ESP_LOGI(TAG, "  Master requests: %.1f/s", stats.master_reqs_per_sec);
  ESP_LOGI(TAG, "  ID responses: %.1f/s", stats.id_answers_per_sec);
  ESP_LOGI(TAG, "  Data bytes: %.1f/s", stats.data_bytes_per_sec);
  ESP_LOGI(TAG, "  Unique IDs seen: %u", stats.unique_ids_seen);
  ESP_LOGI(TAG, "  Bus load: %.1f%%", stats.estimated_bus_load_percent);
  ESP_LOGI(TAG, "  Error rate: %.1f%%", stats.error_rate_percent);
  ESP_LOGI(TAG, "  Stats window: %lld ms", (esp_timer_get_time() - stats.stats_start_time) / 1000);
}
