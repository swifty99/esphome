#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/can_gateway/can_gateway.h"
#include "esphome/core/defines.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

#include "isotp_core.h"

#include <functional>
#include <memory>
#include <vector>

namespace esphome::isotp {

/// Non-owning view of one reassembled message.
///
/// Small and trivially copyable on purpose: it is passed by value into YAML automation lambdas,
/// where a std::vector would cost a heap allocation per message (the same trap that removed
/// can_gateway's on_frame). At 8 bytes it fits any std::function small-buffer optimisation.
struct MessageView {
  const uint8_t *data;
  uint16_t size;
};

/// Base for a C++ consumer of complete messages, typically the diagnostic layer above.
///
/// Deliberately a virtual base rather than a templatized callback, mirroring can_gateway's
/// CanGatewayFrameConsumer: registration must not be able to express an allocating callable, so
/// the no-allocation rule stays a property of the type system rather than a convention.
class IsoTpConsumer {
 public:
  virtual ~IsoTpConsumer() = default;
  /// Called in loop context with a view valid ONLY for this call (see the lifetime rule below).
  virtual void on_message(const MessageView &msg) = 0;
  virtual void on_error(IsoTpError error) {}
};

/// One ISO-TP address pair on one can_gateway port.
///
/// Runs entirely in loop context: frames arrive through can_gateway's observation ring drain and
/// transmits go out through GatewayPort::inject(). Nothing here touches the forwarding fast path.
///
/// **Lifetime rule (normative).** The MessageView handed to a consumer or an automation points
/// into this instance's reassembly buffer and is valid only for the duration of that call. The
/// next transfer overwrites it. Copy out inside the callback if the bytes are needed later.
class IsoTpProtocol : public Component, public can_gateway::CanGatewayFrameConsumer {
 public:
  /// Port and identifiers are required and never change, so they are constructor parameters
  /// rather than setters (project convention: no partially-initialised object).
  IsoTpProtocol(can_gateway::GatewayPort *port, uint32_t tx_id, uint32_t rx_id, bool extended)
      : port_(port), tx_id_(tx_id), rx_id_(rx_id), extended_(extended) {}

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  /// can_gateway subscriber entry point. Loop context, inside the bounded ring drain.
  void on_frame(const can_gateway::FrameView &view) override;

  // -- configuration, all set by codegen before setup() ------------------------------------
  void set_max_message_size(uint16_t size) { this->cfg_.max_message_size = size; }
  void set_block_size(uint8_t block_size) { this->cfg_.block_size = block_size; }
  void set_st_min_raw(uint8_t st_min_raw) { this->cfg_.st_min_raw = st_min_raw; }
  void set_padding(uint8_t byte) {
    this->cfg_.padding_enabled = true;
    this->cfg_.padding_byte = byte;
  }
  void set_address_extension(uint8_t target_address) {
    this->cfg_.address_extension_len = 1;
    this->cfg_.target_address = target_address;
  }
  void set_n_bs_timeout(uint32_t ms) { this->cfg_.n_bs_timeout_ms = ms; }
  void set_n_cr_timeout(uint32_t ms) { this->cfg_.n_cr_timeout_ms = ms; }
  void set_tx_stall_timeout(uint32_t ms) { this->cfg_.tx_stall_timeout_ms = ms; }

  // -- C++ API for the layer above ---------------------------------------------------------
  void set_consumer(IsoTpConsumer *consumer) { this->consumer_ = consumer; }

  /// Queue a message for transmission. Returns false when a send is already in flight (ISO-TP
  /// has no multiplexing, so the caller serialises requests) or the message exceeds capacity.
  bool send(const uint8_t *data, uint16_t size);

  bool is_busy() const { return this->tx_.is_busy(); }

  // -- YAML automation hooks ---------------------------------------------------------------
  // Templatized so both std::function and pointer-sized forwarder structs bind without forcing
  // a heap allocation.
  template<typename F> void add_on_message_callback(F &&callback) {
    this->message_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_error_callback(F &&callback) {
    this->error_callback_.add(std::forward<F>(callback));
  }

  // -- diagnostics -------------------------------------------------------------------------
  uint32_t messages_sent() const { return this->messages_sent_; }
  uint32_t messages_received() const { return this->messages_received_; }
  uint32_t transfers_failed() const { return this->transfers_failed_; }
  uint32_t frames_ignored() const { return this->rx_.ignored(); }

 protected:
  /// At most this many frames leave per loop iteration, bounding the work one instance can do
  /// (N3). STmin usually stops us sooner; this only bites when the peer granted STmin 0.
  static constexpr uint8_t TX_FRAMES_PER_LOOP = 8;

  bool inject_frame_(const Frame &frame);
  void deliver_message_();
  void report_error_(IsoTpError error);

  can_gateway::GatewayPort *port_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  bool extended_;

  IsoTpConfig cfg_;
  TxTransfer tx_;
  RxTransfer rx_;

  std::unique_ptr<uint8_t[]> tx_buffer_;
  std::unique_ptr<uint8_t[]> rx_buffer_;

  IsoTpConsumer *consumer_{nullptr};
  LazyCallbackManager<void(MessageView)> message_callback_;
  LazyCallbackManager<void(IsoTpError)> error_callback_;

  /// A flow control frame the CAN layer refused. Retried from loop() rather than dropped: losing
  /// it stalls the peer until N_Cr expires, which turns a busy microsecond into a lost second.
  Frame pending_fc_{};
  bool has_pending_fc_{false};

  uint32_t messages_sent_{0};
  uint32_t messages_received_{0};
  uint32_t transfers_failed_{0};
};

#ifdef USE_SENSOR
/// Optional diagnostic counters for one instance (B14).
///
/// One polling component serves every counter rather than one per sensor: the counters are read
/// together, and four PollingComponents per ECU adds up quickly once a config talks to several.
class IsoTpDiagnostics : public PollingComponent, public Parented<IsoTpProtocol> {
 public:
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_messages_sent_sensor(sensor::Sensor *s) { this->messages_sent_sensor_ = s; }
  void set_messages_received_sensor(sensor::Sensor *s) { this->messages_received_sensor_ = s; }
  void set_transfers_failed_sensor(sensor::Sensor *s) { this->transfers_failed_sensor_ = s; }
  void set_frames_ignored_sensor(sensor::Sensor *s) { this->frames_ignored_sensor_ = s; }

 protected:
  /// Publish only on change: these counters are mostly idle, and a diagnostic session should not
  /// generate API traffic in proportion to its poll rate.
  static void publish_changed_(sensor::Sensor *s, uint32_t value, uint32_t &last);

  sensor::Sensor *messages_sent_sensor_{nullptr};
  sensor::Sensor *messages_received_sensor_{nullptr};
  sensor::Sensor *transfers_failed_sensor_{nullptr};
  sensor::Sensor *frames_ignored_sensor_{nullptr};

  uint32_t last_messages_sent_{UINT32_MAX};
  uint32_t last_messages_received_{UINT32_MAX};
  uint32_t last_transfers_failed_{UINT32_MAX};
  uint32_t last_frames_ignored_{UINT32_MAX};
};
#endif

/// `isotp.send` action.
///
/// Tier 1, loop context: this is the control plane, not a per-frame path, so the allocation a
/// templatable payload costs (the lambda returns a vector by value) is paid only when an
/// automation actually fires. A static payload is built once at setup and never reallocated.
template<typename... Ts> class SendAction : public Action<Ts...>, public Parented<IsoTpProtocol> {
 public:
  void set_data_template(std::function<std::vector<uint8_t>(Ts...)> func) {
    this->data_func_ = std::move(func);
    this->is_static_ = false;
  }
  void set_data_static(const std::vector<uint8_t> &data) {
    this->data_static_ = data;
    this->is_static_ = true;
  }

  void play(const Ts &...x) override {
    if (this->is_static_) {
      this->parent_->send(this->data_static_.data(), static_cast<uint16_t>(this->data_static_.size()));
      return;
    }
    const std::vector<uint8_t> data = this->data_func_(x...);
    this->parent_->send(data.data(), static_cast<uint16_t>(data.size()));
  }

 protected:
  bool is_static_{true};
  std::function<std::vector<uint8_t>(Ts...)> data_func_{};
  std::vector<uint8_t> data_static_{};
};

}  // namespace esphome::isotp
