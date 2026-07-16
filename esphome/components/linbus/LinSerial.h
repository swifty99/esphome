#pragma once

#include "driver/uart.h"
#include "esp_err.h"
#include <cstdint>
#include <cstddef>

// ESP32 dependant
// INTENTIONAL: High-level UART driver ONLY. No custom LL ISR / uart_ll_* usage.

class LinSerial {
 public:
  LinSerial(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate);
  ~LinSerial();

  // Configure + install the UART driver. Returns the driver error instead of aborting,
  // so a bad config (e.g. UART already owned by the logger) can mark_failed() rather than
  // bootloop. Must be called once before use. See UPGRADES.md H5.
  esp_err_t init();

  bool available();
  bool readByte(uint8_t &byte);
  bool writeBytes(const uint8_t *data, size_t len);
  void flush();
  uart_port_t getPort() const { return uart_num_; }

  // Sende LIN Header: Break (HW oder Fallback), Sync (0x55) und PID (ID + Parity)
  // id: 0..63 (nur 6 Bit), break_bits typisch 13
  // use_hw_break=true nutzt UART HW-Break (empfohlen)
  bool sendHeader(uint8_t id, bool use_hw_break = true, uint8_t break_bits = 13);

  // Send LIN break signal. Returns false on TX error/timeout (bounded wait). See UPGRADES.md M3.
  bool sendBreak();

  // Event queue accessor (driver-owned ISR pushes events here)
  QueueHandle_t getEventQueue() const { return uart_queue_; }

 private:
  uart_port_t uart_num_;
  int tx_pin_;
  int rx_pin_;
  int baud_rate_;

  QueueHandle_t uart_queue_{nullptr};  // driver event queue
};
