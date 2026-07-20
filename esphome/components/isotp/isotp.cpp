#include "isotp.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::isotp {

static const char *const TAG = "isotp";

static const char *error_to_string(IsoTpError error) {
  switch (error) {
    case IsoTpError::NONE:
      return "none";
    case IsoTpError::TIMEOUT_FC:
      return "flow control timeout (N_Bs)";
    case IsoTpError::TIMEOUT_CF:
      return "consecutive frame timeout (N_Cr)";
    case IsoTpError::WRONG_SEQUENCE:
      return "wrong sequence number";
    case IsoTpError::OVERFLOW_LOCAL:
      return "message larger than our buffer";
    case IsoTpError::OVERFLOW_REMOTE:
      return "peer reported overflow";
    case IsoTpError::TX_STALLED:
      return "transmit stalled";
    case IsoTpError::ABORTED:
      return "aborted";
  }
  return "unknown";
}

void IsoTpProtocol::setup() {
  // One allocation each, here and never again: everything after setup() is allocation-free so a
  // long-running diagnostic session cannot fragment the heap.
  this->tx_buffer_ = std::make_unique<uint8_t[]>(this->cfg_.max_message_size);
  this->rx_buffer_ = std::make_unique<uint8_t[]>(this->cfg_.max_message_size);
  this->tx_.init(this->tx_buffer_.get(), this->cfg_.max_message_size);
  this->rx_.init(this->rx_buffer_.get(), this->cfg_.max_message_size);
}

void IsoTpProtocol::on_frame(const can_gateway::FrameView &view) {
  // view.data dies the moment this returns, so every path below either consumes the bytes now or
  // copies them into our own buffer. Nothing retains the pointer.
  const PciInfo info = decode_pci(this->cfg_, view.data, view.dlc);

  if (info.type == PciType::FLOW_CONTROL) {
    // Belongs to the send machine. Its failure (a peer overflow) parks in the machine and is
    // reported from the next loop(), which is why on_flow_control() returns nothing.
    this->tx_.on_flow_control(info, millis());
    return;
  }

  Frame out{};
  const RxAction action = this->rx_.on_frame(this->cfg_, view.data, view.dlc, millis(), &out);
  switch (action) {
    case RxAction::SEND_FLOW_CONTROL:
      // Sent inline rather than deferred to the next loop tick: a deferred flow control frame
      // adds a whole loop period to every block, which on a long response is the difference
      // between a brisk transfer and a visibly slow one. inject() is loop context and short, and
      // the drain we are inside is already loop context, so this is legal.
      if (!this->inject_frame_(out)) {
        this->pending_fc_ = out;
        this->has_pending_fc_ = true;
      }
      break;
    case RxAction::MESSAGE_COMPLETE:
      this->deliver_message_();
      break;
    case RxAction::FAILED:
      this->report_error_(this->rx_.error());
      break;
    case RxAction::NOTHING:
      break;
  }
}

void IsoTpProtocol::loop() {
  const uint32_t now = millis();

  // A refused flow control frame is the most time-critical thing we can be holding: until it
  // lands the peer is blocked, so it goes out ahead of our own transmit work.
  if (this->has_pending_fc_ && this->inject_frame_(this->pending_fc_))
    this->has_pending_fc_ = false;

  if (this->rx_.poll(this->cfg_, now) == RxAction::FAILED)
    this->report_error_(this->rx_.error());

  for (uint8_t i = 0; i < TX_FRAMES_PER_LOOP; i++) {
    Frame frame{};
    const TxAction action = this->tx_.poll(this->cfg_, now, &frame);
    if (action == TxAction::SEND_FRAME) {
      // A refused frame is backpressure, not an error: without confirm_sent() the machine
      // re-offers the identical frame next time, sequence number and all.
      if (!this->inject_frame_(frame))
        break;
      this->tx_.confirm_sent(this->cfg_, now);
      continue;
    }
    if (action == TxAction::COMPLETE) {
      this->messages_sent_++;
    } else if (action == TxAction::FAILED) {
      this->report_error_(this->tx_.error());
    }
    break;  // NOTHING, COMPLETE and FAILED all end this loop iteration's transmit work.
  }
}

bool IsoTpProtocol::send(const uint8_t *data, uint16_t size) {
  if (!this->tx_.begin(data, size, millis())) {
    ESP_LOGW(TAG, "send rejected: %s",
             this->tx_.is_busy() ? "a transfer is already in flight" : "message exceeds max_message_size");
    return false;
  }
  return true;
}

bool IsoTpProtocol::inject_frame_(const Frame &frame) {
  return this->port_->inject(this->tx_id_, this->extended_, false, frame.data, frame.dlc);
}

void IsoTpProtocol::deliver_message_() {
  this->messages_received_++;
  const MessageView msg{this->rx_.data(), this->rx_.size()};
  ESP_LOGV(TAG, "received %u bytes", msg.size);
  // Both consumers run inside can_gateway's bounded drain, so they are expected to be brief:
  // parse and buffer, act on the next tick.
  if (this->consumer_ != nullptr)
    this->consumer_->on_message(msg);
  this->message_callback_.call(msg);
}

void IsoTpProtocol::report_error_(IsoTpError error) {
  this->transfers_failed_++;
  ESP_LOGW(TAG, "transfer failed: %s", error_to_string(error));
  if (this->consumer_ != nullptr)
    this->consumer_->on_error(error);
  this->error_callback_.call(error);
}

void IsoTpProtocol::dump_config() {
  ESP_LOGCONFIG(TAG, "ISO-TP:");
  ESP_LOGCONFIG(TAG, "  Request ID: 0x%03" PRIX32 " (%s)", this->tx_id_, this->extended_ ? "29-bit" : "11-bit");
  ESP_LOGCONFIG(TAG, "  Response ID: 0x%03" PRIX32, this->rx_id_);
  if (this->cfg_.address_extension_len != 0)
    ESP_LOGCONFIG(TAG, "  Extended addressing, target 0x%02X", this->cfg_.target_address);
  ESP_LOGCONFIG(TAG, "  Max message size: %u bytes", this->cfg_.max_message_size);
  ESP_LOGCONFIG(TAG, "  Block size granted: %u frames", this->cfg_.block_size);
  ESP_LOGCONFIG(TAG, "  STmin requested: %" PRIu32 " ms (raw 0x%02X)", st_min_to_ms(this->cfg_.st_min_raw),
                this->cfg_.st_min_raw);
  if (this->cfg_.padding_enabled) {
    ESP_LOGCONFIG(TAG, "  Padding: 0x%02X", this->cfg_.padding_byte);
  } else {
    ESP_LOGCONFIG(TAG, "  Padding: disabled");
  }
  ESP_LOGCONFIG(TAG, "  Timeouts: N_Bs %" PRIu32 " ms, N_Cr %" PRIu32 " ms, tx stall %" PRIu32 " ms",
                this->cfg_.n_bs_timeout_ms, this->cfg_.n_cr_timeout_ms, this->cfg_.tx_stall_timeout_ms);
}

#ifdef USE_SENSOR
void IsoTpDiagnostics::publish_changed_(sensor::Sensor *s, uint32_t value, uint32_t &last) {
  if (s == nullptr || value == last)
    return;
  last = value;
  s->publish_state(static_cast<float>(value));
}

void IsoTpDiagnostics::update() {
  IsoTpProtocol *p = this->parent_;
  publish_changed_(this->messages_sent_sensor_, p->messages_sent(), this->last_messages_sent_);
  publish_changed_(this->messages_received_sensor_, p->messages_received(), this->last_messages_received_);
  publish_changed_(this->transfers_failed_sensor_, p->transfers_failed(), this->last_transfers_failed_);
  publish_changed_(this->frames_ignored_sensor_, p->frames_ignored(), this->last_frames_ignored_);
}

void IsoTpDiagnostics::dump_config() {
  ESP_LOGCONFIG(TAG, "ISO-TP diagnostics:");
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Messages sent", this->messages_sent_sensor_);
  LOG_SENSOR("  ", "Messages received", this->messages_received_sensor_);
  LOG_SENSOR("  ", "Transfers failed", this->transfers_failed_sensor_);
  LOG_SENSOR("  ", "Frames ignored", this->frames_ignored_sensor_);
}
#endif

}  // namespace esphome::isotp
