#include "LinSerial.h"
#include "driver/uart.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <cstring>
#include "esp_timer.h"  // For ets_delay_us or alternative delay

#include "esp_intr_alloc.h"

static const char *TAG = "LinSerial";

LinSerial::LinSerial(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate)
    : uart_num_(uart_num), tx_pin_(tx_pin), rx_pin_(rx_pin), baud_rate_(baud_rate) {}

esp_err_t LinSerial::init() {
  uart_config_t uart_config = {.baud_rate = baud_rate_,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                               .rx_flow_ctrl_thresh = 0,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
                               .source_clk = UART_SCLK_DEFAULT};
#else
                               .source_clk = UART_SCLK_APB};
#endif

  esp_err_t err = uart_param_config(uart_num_, &uart_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
    return err;
  }
  err = uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
    return err;
  }

  // Install driver WITH event queue (RX only buffer)
  const int RX_BUF = 512;
  const int TX_BUF = 0;
  const int QUEUE_SZ = 20;
  err = uart_driver_install(uart_num_, RX_BUF, TX_BUF, QUEUE_SZ, &uart_queue_, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install failed on port %d: %s", uart_num_, esp_err_to_name(err));
    return err;
  }

  // Faster delivery (1 char timeout + threshold 1)
  uart_set_rx_full_threshold(uart_num_, 1);
  uart_set_rx_timeout(uart_num_, 1);
  return ESP_OK;
}

LinSerial::~LinSerial() {
  if (uart_queue_) {
    uart_driver_delete(uart_num_);
    uart_queue_ = nullptr;
  }
}

bool LinSerial::sendBreak() {
  // Set baud rate to half speed for the break signal, trusted and works good
  uint32_t break_baud_rate = baud_rate_ / 2;
  if (uart_set_baudrate(uart_num_, break_baud_rate) != ESP_OK)
    return false;

  // Send 0x00 byte to create the break condition
  uint8_t break_byte = 0x00;
  int written = uart_write_bytes(uart_num_, (const char *) &break_byte, 1);
  // Bounded wait so a wedged transceiver (bus held dominant) can't hang the main loop -> WDT.
  esp_err_t wait_err = uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(20));

  // Always restore original baud rate, even if the break TX failed.
  uart_set_baudrate(uart_num_, baud_rate_);
  return written == 1 && wait_err == ESP_OK;
}

bool LinSerial::available() {
  size_t bytes = 0;
  ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num_, &bytes));
  return bytes > 0;
}

bool LinSerial::readByte(uint8_t &byte) {
  int len = uart_read_bytes(uart_num_, &byte, 1, 0);
  return len == 1;
}

bool LinSerial::writeBytes(const uint8_t *data, size_t len) {
  int written = uart_write_bytes(uart_num_, (const char *) data, len);
  return written == (int) len;
}

void LinSerial::flush() { uart_flush_input(uart_num_); }
