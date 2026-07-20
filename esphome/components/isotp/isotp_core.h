#pragma once

/// Freestanding ISO 15765-2 (ISO-TP) transport logic: frame codec plus the send and receive
/// state machines. Deliberately has no ESPHome, ESP-IDF or can_gateway dependency so the whole
/// protocol can be exercised on the host (see the isotp host test suite).
///
/// Role is **tester / client** only: this segments requests and reassembles responses. It never
/// emulates an ECU.
///
/// Everything here runs in **loop context**. ISO-TP deadlines (N_Bs and N_Cr around 1000 ms) are
/// two orders of magnitude above a typical ~16 ms ESPHome loop, and as the receiver we author the
/// flow control frame, so we set the pace ourselves and cannot be outrun. Nothing in this header
/// needs to be interrupt-safe or IRAM-resident.
///
/// The state machines never transmit and never allocate. They are driven by two calls:
///   - `on_frame()` when a CAN frame arrives, and
///   - `poll()` once per loop iteration,
/// each of which returns an action telling the caller what to do. Buffers are supplied by the
/// caller through `init()`, which keeps allocation in the component's `setup()` and keeps this
/// header pure.

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace esphome::isotp {

/// Classic CAN carries 8 data bytes. ISO-TP over CAN FD is out of scope.
static constexpr uint8_t FRAME_LEN = 8;

/// Largest message expressible without the ISO 15765-2:2016 escape sequence (12-bit length field).
static constexpr uint16_t MAX_CLASSIC_MESSAGE_SIZE = 4095;

/// Protocol Control Information nibble, the high nibble of the first PCI byte.
enum class PciType : uint8_t {
  SINGLE = 0x0,
  FIRST = 0x1,
  CONSECUTIVE = 0x2,
  FLOW_CONTROL = 0x3,
  INVALID = 0xFF,
};

/// FlowStatus, the low nibble of a flow control frame's first PCI byte.
///
/// The standard calls the third value "Overflow". It is spelled BUFFER_OVERFLOW here because
/// <math.h> defines an object-like OVERFLOW macro on both macOS and glibc, so the plain name
/// breaks any translation unit that happens to include math.h first.
enum class FlowStatus : uint8_t {
  CONTINUE = 0x0,         ///< clear to send
  WAIT = 0x1,             ///< pause, restart the N_Bs timer
  BUFFER_OVERFLOW = 0x2,  ///< receiver cannot hold the declared message
};

/// Unsigned modular subtraction, so a millisecond counter wrapping at 2^32 (about 49 days) needs
/// no special case.
inline uint32_t elapsed_ms(uint32_t now, uint32_t start) { return now - start; }

enum class IsoTpError : uint8_t {
  NONE = 0,
  TIMEOUT_FC,       ///< N_Bs: no flow control after a first frame or a completed block
  TIMEOUT_CF,       ///< N_Cr: no next consecutive frame
  WRONG_SEQUENCE,   ///< a consecutive frame arrived out of order
  OVERFLOW_LOCAL,   ///< their message is larger than our reassembly buffer
  OVERFLOW_REMOTE,  ///< their flow control reported overflow
  TX_STALLED,       ///< the CAN layer refused our frame for too long
  ABORTED,          ///< cancelled by the layer above
};

/// One CAN frame's payload, as produced by the codec. The caller supplies the identifier.
struct Frame {
  uint8_t data[FRAME_LEN];
  uint8_t dlc;
};

/// Everything the codec and the state machines need to know about one address pair.
/// `address_extension_len` is 1 for extended addressing (the first payload byte carries the
/// target address, costing one data byte in every frame) and 0 otherwise, which lets the offset
/// be applied once rather than branched on throughout.
struct IsoTpConfig {
  uint16_t max_message_size{256};
  uint8_t address_extension_len{0};
  uint8_t target_address{0};
  uint8_t block_size{8};  ///< frames per block we grant as receiver; never 0 (see below)
  uint8_t st_min_raw{0};  ///< STmin we request, in the raw ISO encoding
  uint8_t padding_byte{0xCC};
  bool padding_enabled{false};
  uint32_t n_bs_timeout_ms{1000};
  uint32_t n_cr_timeout_ms{1000};
  uint32_t tx_stall_timeout_ms{1000};
};

/// Decoded first-PCI-byte information. Which members are meaningful depends on `type`.
struct PciInfo {
  PciType type{PciType::INVALID};
  uint16_t length{0};                       ///< SINGLE, FIRST: message length
  uint8_t sequence{0};                      ///< CONSECUTIVE: sequence number 0-15
  FlowStatus status{FlowStatus::CONTINUE};  ///< FLOW_CONTROL
  uint8_t block_size{0};                    ///< FLOW_CONTROL
  uint8_t st_min_raw{0};                    ///< FLOW_CONTROL
  uint8_t payload_offset{0};                ///< index of the first data byte within the frame
  uint8_t payload_len{0};                   ///< data bytes carried by this frame
};

// -------------------------------------------------------------------------------------------
// STmin
// -------------------------------------------------------------------------------------------

/// Convert the raw ISO STmin encoding to whole milliseconds.
///
/// 0x00-0x7F are milliseconds directly. 0xF1-0xF9 encode 100-900 microseconds, which is below a
/// loop period, so they collapse to 1 ms: STmin is a *minimum*, and waiting longer is conforming,
/// merely slower. Every other value is reserved; the standard says to treat a reserved value as
/// the maximum, so it becomes 127 ms rather than being rejected.
inline uint32_t st_min_to_ms(uint8_t raw) {
  if (raw <= 0x7F)
    return raw;
  if (raw >= 0xF1 && raw <= 0xF9)
    return 1;
  return 127;
}

/// True when `raw` is a value this component is willing to *request* (config validation mirrors
/// this). Reserved encodings are accepted on receive but never emitted.
inline bool st_min_is_valid(uint8_t raw) { return raw <= 0x7F || (raw >= 0xF1 && raw <= 0xF9); }

// -------------------------------------------------------------------------------------------
// Frame codec
// -------------------------------------------------------------------------------------------

/// Number of data bytes a frame of the given type can carry under this configuration.
inline uint8_t single_frame_capacity(const IsoTpConfig &cfg) {
  return static_cast<uint8_t>(FRAME_LEN - 1 - cfg.address_extension_len);
}
inline uint8_t first_frame_capacity(const IsoTpConfig &cfg) {
  return static_cast<uint8_t>(FRAME_LEN - 2 - cfg.address_extension_len);
}
inline uint8_t consecutive_frame_capacity(const IsoTpConfig &cfg) {
  return static_cast<uint8_t>(FRAME_LEN - 1 - cfg.address_extension_len);
}

/// Apply the addressing prefix and, if configured, pad the frame out to 8 bytes.
/// Padding matters: many ECUs reject short frames outright.
inline void finish_frame(const IsoTpConfig &cfg, Frame *out, uint8_t used) {
  if (cfg.address_extension_len != 0)
    out->data[0] = cfg.target_address;
  if (cfg.padding_enabled) {
    for (uint8_t i = used; i < FRAME_LEN; i++)
      out->data[i] = cfg.padding_byte;
    out->dlc = FRAME_LEN;
  } else {
    out->dlc = used;
  }
}

/// Encode a complete message that fits in one frame. Returns false when it does not fit.
inline bool encode_single_frame(const IsoTpConfig &cfg, const uint8_t *data, uint16_t size, Frame *out) {
  if (size == 0 || size > single_frame_capacity(cfg))
    return false;
  const uint8_t pci = cfg.address_extension_len;
  out->data[pci] = static_cast<uint8_t>(static_cast<uint8_t>(PciType::SINGLE) << 4 | size);
  std::memcpy(&out->data[pci + 1], data, size);
  finish_frame(cfg, out, static_cast<uint8_t>(pci + 1 + size));
  return true;
}

/// Encode the first frame of a multi-frame message. Returns the number of data bytes consumed.
inline uint8_t encode_first_frame(const IsoTpConfig &cfg, const uint8_t *data, uint16_t total_size, Frame *out) {
  const uint8_t pci = cfg.address_extension_len;
  out->data[pci] = static_cast<uint8_t>(static_cast<uint8_t>(PciType::FIRST) << 4 | ((total_size >> 8) & 0x0F));
  out->data[pci + 1] = static_cast<uint8_t>(total_size & 0xFF);
  const uint8_t take = first_frame_capacity(cfg);
  std::memcpy(&out->data[pci + 2], data, take);
  // A first frame is always full: the message is by definition longer than one frame holds.
  finish_frame(cfg, out, static_cast<uint8_t>(pci + 2 + take));
  return take;
}

/// Encode one consecutive frame. `remaining` is how many message bytes are still unsent.
/// Returns the number of data bytes consumed, which is short only on the final frame.
inline uint8_t encode_consecutive_frame(const IsoTpConfig &cfg, const uint8_t *data, uint16_t remaining,
                                        uint8_t sequence, Frame *out) {
  const uint8_t pci = cfg.address_extension_len;
  out->data[pci] = static_cast<uint8_t>(static_cast<uint8_t>(PciType::CONSECUTIVE) << 4 | (sequence & 0x0F));
  const uint8_t cap = consecutive_frame_capacity(cfg);
  const uint8_t take = remaining < cap ? static_cast<uint8_t>(remaining) : cap;
  std::memcpy(&out->data[pci + 1], data, take);
  finish_frame(cfg, out, static_cast<uint8_t>(pci + 1 + take));
  return take;
}

inline void encode_flow_control(const IsoTpConfig &cfg, FlowStatus status, uint8_t block_size, uint8_t st_min_raw,
                                Frame *out) {
  const uint8_t pci = cfg.address_extension_len;
  out->data[pci] =
      static_cast<uint8_t>(static_cast<uint8_t>(PciType::FLOW_CONTROL) << 4 | static_cast<uint8_t>(status));
  out->data[pci + 1] = block_size;
  out->data[pci + 2] = st_min_raw;
  finish_frame(cfg, out, static_cast<uint8_t>(pci + 3));
}

/// Decode the PCI of a received frame. Returns `type == INVALID` for anything malformed: a frame
/// too short for its own PCI, an unknown type nibble, or a single frame whose declared length does
/// not fit. Received frames are accepted at any DLC, so padding is never assumed on input.
inline PciInfo decode_pci(const IsoTpConfig &cfg, const uint8_t *data, uint8_t dlc) {
  PciInfo info;
  const uint8_t pci = cfg.address_extension_len;
  if (dlc < static_cast<uint8_t>(pci + 1))
    return info;
  const uint8_t first = data[pci];
  switch (first >> 4) {
    case static_cast<uint8_t>(PciType::SINGLE): {
      const uint8_t len = first & 0x0F;
      if (len == 0 || len > single_frame_capacity(cfg) || dlc < static_cast<uint8_t>(pci + 1 + len))
        return info;
      info.type = PciType::SINGLE;
      info.length = len;
      info.payload_offset = static_cast<uint8_t>(pci + 1);
      info.payload_len = len;
      break;
    }
    case static_cast<uint8_t>(PciType::FIRST): {
      if (dlc < static_cast<uint8_t>(pci + 2))
        return info;
      const uint16_t len = static_cast<uint16_t>((first & 0x0F) << 8 | data[pci + 1]);
      // A first frame must declare more than a single frame could have carried; anything less is
      // malformed, and treating it as valid would let a sender open a reassembly it never closes.
      if (len <= single_frame_capacity(cfg))
        return info;
      info.type = PciType::FIRST;
      info.length = len;
      info.payload_offset = static_cast<uint8_t>(pci + 2);
      info.payload_len = first_frame_capacity(cfg);
      if (dlc < static_cast<uint8_t>(info.payload_offset + info.payload_len))
        return PciInfo{};
      break;
    }
    case static_cast<uint8_t>(PciType::CONSECUTIVE): {
      info.type = PciType::CONSECUTIVE;
      info.sequence = first & 0x0F;
      info.payload_offset = static_cast<uint8_t>(pci + 1);
      info.payload_len = static_cast<uint8_t>(dlc - info.payload_offset);
      break;
    }
    case static_cast<uint8_t>(PciType::FLOW_CONTROL): {
      if (dlc < static_cast<uint8_t>(pci + 3))
        return info;
      const uint8_t status = first & 0x0F;
      if (status > static_cast<uint8_t>(FlowStatus::BUFFER_OVERFLOW))
        return info;
      info.type = PciType::FLOW_CONTROL;
      info.status = static_cast<FlowStatus>(status);
      info.block_size = data[pci + 1];
      info.st_min_raw = data[pci + 2];
      break;
    }
    default:
      return info;
  }
  return info;
}

// -------------------------------------------------------------------------------------------
// Send state machine
// -------------------------------------------------------------------------------------------

enum class TxAction : uint8_t {
  NOTHING,     ///< idle, or waiting on a timer or the peer
  SEND_FRAME,  ///< `out` holds a frame to transmit; call confirm_sent() once it is accepted
  COMPLETE,    ///< the message has been fully transmitted
  FAILED,      ///< see error()
};

/// Segments one outgoing message. Holds a copy of it: a transfer outlives the caller's stack
/// frame by many loop iterations, so borrowing the caller's pointer would be a lifetime trap.
/// The copy is one memcpy per *message*, not per frame, so it does not scale with frame count.
class TxTransfer {
 public:
  /// Supply the send buffer. Called once, from the component's setup().
  void init(uint8_t *buffer, uint16_t capacity) {
    this->buffer_ = buffer;
    this->capacity_ = capacity;
  }

  bool is_busy() const { return this->state_ != State::IDLE; }
  IsoTpError error() const { return this->error_; }

  /// Start sending. Returns false when a transfer is already in flight (ISO-TP has no
  /// multiplexing, so the layer above serialises requests) or the message does not fit.
  bool begin(const uint8_t *data, uint16_t size, uint32_t now_ms) {
    if (this->state_ != State::IDLE || size == 0 || size > this->capacity_)
      return false;
    std::memcpy(this->buffer_, data, size);
    this->size_ = size;
    this->sent_ = 0;
    this->sequence_ = 0;
    this->block_remaining_ = 0;
    this->error_ = IsoTpError::NONE;
    this->state_ = State::SEND_FIRST;
    this->timer_start_ = now_ms;
    this->stall_start_ = now_ms;
    return true;
  }

  void abort() {
    if (this->state_ != State::IDLE) {
      this->error_ = IsoTpError::ABORTED;
      this->state_ = State::IDLE;
    }
  }

  /// Called once per loop iteration. Produces at most one frame per call, which is what bounds
  /// the component's per-iteration work.
  TxAction poll(const IsoTpConfig &cfg, uint32_t now_ms, Frame *out) {
    switch (this->state_) {
      case State::IDLE:
        return TxAction::NOTHING;

      case State::SEND_FIRST:
        if (this->size_ <= single_frame_capacity(cfg)) {
          encode_single_frame(cfg, this->buffer_, this->size_, out);
          return this->offer_(cfg, now_ms, Pending::SINGLE);
        }
        encode_first_frame(cfg, this->buffer_, this->size_, out);
        return this->offer_(cfg, now_ms, Pending::FIRST);

      case State::WAIT_FC:
        if (elapsed_ms(now_ms, this->timer_start_) >= cfg.n_bs_timeout_ms) {
          this->fail_(IsoTpError::TIMEOUT_FC);
          return this->take_failure_();
        }
        return TxAction::NOTHING;

      case State::SEND_CF: {
        if (this->st_min_ms_ != 0 && elapsed_ms(now_ms, this->last_cf_ms_) < this->st_min_ms_)
          return TxAction::NOTHING;
        // Work out this frame's sequence number WITHOUT committing it. The CAN layer may refuse
        // the frame, and B12 requires the next poll to re-offer exactly the same one; because
        // sequence_ has not moved, re-encoding here is idempotent. Committing early was a real
        // bug: one refused inject() would make the ECU see a gap and abort with WRONG_SEQUENCE.
        this->pending_sequence_ = static_cast<uint8_t>((this->sequence_ + 1) & 0x0F);
        this->pending_take_ =
            encode_consecutive_frame(cfg, this->buffer_ + this->sent_, static_cast<uint16_t>(this->size_ - this->sent_),
                                     this->pending_sequence_, out);
        return this->offer_(cfg, now_ms, Pending::CONSECUTIVE);
      }

      case State::DONE:
        this->state_ = State::IDLE;
        return TxAction::COMPLETE;

      case State::FAILED:
        return this->take_failure_();
    }
    return TxAction::NOTHING;
  }

  /// Confirm that the frame handed out by the last SEND_FRAME was accepted by the CAN layer.
  ///
  /// Deliberately separate from poll(): can_gateway's inject() returns false when its queue is
  /// full or the port is bus-off, and that is a "try again next loop", not an error. Until this
  /// is called the sequence number does not advance and the same frame is re-offered, so a busy
  /// bus costs throughput rather than correctness.
  void confirm_sent(const IsoTpConfig &cfg, uint32_t now_ms) {
    switch (this->pending_) {
      case Pending::NONE:
        return;
      case Pending::SINGLE:
        this->state_ = State::DONE;
        break;
      case Pending::FIRST:
        this->sent_ = first_frame_capacity(cfg);
        this->state_ = State::WAIT_FC;
        this->timer_start_ = now_ms;
        break;
      case Pending::CONSECUTIVE:
        this->sequence_ = this->pending_sequence_;  // commit only now that it is really on the bus
        this->sent_ = static_cast<uint16_t>(this->sent_ + this->pending_take_);
        this->last_cf_ms_ = now_ms;
        if (this->sent_ >= this->size_) {
          this->state_ = State::DONE;
        } else if (this->block_remaining_ != 0) {
          this->block_remaining_--;
          if (this->block_remaining_ == 0) {
            // Block exhausted: the peer owes us another flow control frame before we may continue.
            this->state_ = State::WAIT_FC;
            this->timer_start_ = now_ms;
          }
        }
        break;
    }
    this->pending_ = Pending::NONE;
  }

  /// Feed a received flow control frame. Frames of other types are not this machine's business.
  void on_flow_control(const PciInfo &info, uint32_t now_ms) {
    if (this->state_ != State::WAIT_FC)
      return;
    switch (info.status) {
      case FlowStatus::CONTINUE:
        // block_size 0 from the peer means "send everything without further flow control", which
        // is legal for us as sender: we are the one pacing, and we send at most one frame a loop.
        this->block_remaining_ = info.block_size;
        this->st_min_ms_ = st_min_to_ms(info.st_min_raw);
        this->last_cf_ms_ = now_ms - this->st_min_ms_;  // first frame of a block need not wait
        this->state_ = State::SEND_CF;
        break;
      case FlowStatus::WAIT:
        this->timer_start_ = now_ms;  // restart N_Bs, keep waiting
        break;
      case FlowStatus::BUFFER_OVERFLOW:
        this->fail_(IsoTpError::OVERFLOW_REMOTE);
        break;
    }
  }

 protected:
  enum class State : uint8_t { IDLE, SEND_FIRST, WAIT_FC, SEND_CF, DONE, FAILED };
  enum class Pending : uint8_t { NONE, SINGLE, FIRST, CONSECUTIVE };

  /// Record a failure. Deliberately returns nothing: a failure detected while handling an
  /// incoming frame (a peer OVERFLOW flow control, say) has no caller to return an action to, so
  /// it parks in State::FAILED and the next poll() delivers it. Previously this set State::IDLE
  /// and returned an action that on_flow_control() discarded, so a remote overflow aborted the
  /// transfer without ever telling anyone.
  void fail_(IsoTpError error) {
    this->error_ = error;
    this->state_ = State::FAILED;
    this->pending_ = Pending::NONE;
  }

  /// The single exit for a recorded failure.
  TxAction take_failure_() {
    this->state_ = State::IDLE;
    return TxAction::FAILED;
  }

  /// Hand a frame to the caller, and police B12.
  ///
  /// The stall timer starts when a frame is FIRST offered and restarts whenever a different frame
  /// is offered, so it measures how long the CAN layer has been refusing *this* frame. Timing
  /// from the last successful send instead was a real bug: a legal exchange where the ECU sends
  /// FC WAIT and then CONTINUE later than tx_stall_timeout_ms would fail with TX_STALLED even
  /// though nothing was ever refused.
  TxAction offer_(const IsoTpConfig &cfg, uint32_t now_ms, Pending kind) {
    if (this->pending_ != kind) {
      this->pending_ = kind;
      this->stall_start_ = now_ms;
      return TxAction::SEND_FRAME;
    }
    if (elapsed_ms(now_ms, this->stall_start_) >= cfg.tx_stall_timeout_ms) {
      this->fail_(IsoTpError::TX_STALLED);
      return this->take_failure_();
    }
    return TxAction::SEND_FRAME;
  }

  uint8_t *buffer_{nullptr};
  uint16_t capacity_{0};
  uint16_t size_{0};
  uint16_t sent_{0};
  uint32_t timer_start_{0};
  uint32_t stall_start_{0};
  uint32_t last_cf_ms_{0};
  uint32_t st_min_ms_{0};
  State state_{State::IDLE};
  Pending pending_{Pending::NONE};
  IsoTpError error_{IsoTpError::NONE};
  uint8_t sequence_{0};
  uint8_t pending_sequence_{0};  ///< sequence of the frame currently offered, not yet committed
  uint8_t block_remaining_{0};
  uint8_t pending_take_{0};
};

// -------------------------------------------------------------------------------------------
// Receive state machine
// -------------------------------------------------------------------------------------------

enum class RxAction : uint8_t {
  NOTHING,            ///< nothing to do, or the frame was ignored
  SEND_FLOW_CONTROL,  ///< `out` holds a flow control frame to transmit
  MESSAGE_COMPLETE,   ///< a full message is available via data()/size()
  FAILED,             ///< see error()
};

/// Reassembles one incoming message into a caller-supplied buffer.
class RxTransfer {
 public:
  void init(uint8_t *buffer, uint16_t capacity) {
    this->buffer_ = buffer;
    this->capacity_ = capacity;
  }

  const uint8_t *data() const { return this->buffer_; }
  uint16_t size() const { return this->size_; }
  IsoTpError error() const { return this->error_; }
  bool is_busy() const { return this->state_ == State::RECEIVING; }
  /// Frames counted but deliberately not acted on (B9): a stray consecutive frame, a malformed
  /// PCI. Surfaced as a diagnostic rather than an error because a shared bus carries traffic that
  /// is simply not ours.
  uint32_t ignored() const { return this->ignored_; }

  void reset() {
    this->state_ = State::IDLE;
    this->size_ = 0;
  }

  /// Feed one received frame. Returns what the caller should do about it.
  RxAction on_frame(const IsoTpConfig &cfg, const uint8_t *data, uint8_t dlc, uint32_t now_ms, Frame *out) {
    const PciInfo info = decode_pci(cfg, data, dlc);
    switch (info.type) {
      case PciType::SINGLE:
        // A single frame is self-contained, so it is delivered whatever else was in progress.
        std::memcpy(this->buffer_, &data[info.payload_offset], info.payload_len);
        this->size_ = info.payload_len;
        this->state_ = State::IDLE;
        return RxAction::MESSAGE_COMPLETE;

      case PciType::FIRST: {
        // A repeated first frame restarts reassembly: it most likely means the peer retried
        // because our flow control was lost, and refusing it would deadlock the session (B9).
        if (info.length > this->capacity_) {
          // Answer overflow without touching the buffer; the peer must not start sending.
          encode_flow_control(cfg, FlowStatus::BUFFER_OVERFLOW, 0, 0, out);
          this->state_ = State::IDLE;
          this->error_ = IsoTpError::OVERFLOW_LOCAL;
          return RxAction::SEND_FLOW_CONTROL;
        }
        std::memcpy(this->buffer_, &data[info.payload_offset], info.payload_len);
        this->size_ = info.payload_len;
        this->expected_ = info.length;
        this->sequence_ = 0;
        this->state_ = State::RECEIVING;
        this->block_remaining_ = cfg.block_size;
        this->timer_start_ = now_ms;
        encode_flow_control(cfg, FlowStatus::CONTINUE, cfg.block_size, cfg.st_min_raw, out);
        return RxAction::SEND_FLOW_CONTROL;
      }

      case PciType::CONSECUTIVE: {
        if (this->state_ != State::RECEIVING) {
          this->ignored_++;
          return RxAction::NOTHING;
        }
        const uint8_t want = static_cast<uint8_t>((this->sequence_ + 1) & 0x0F);
        if (info.sequence != want) {
          // Never resynchronise silently: for a tester, a corrupt response must be visible.
          this->state_ = State::IDLE;
          this->error_ = IsoTpError::WRONG_SEQUENCE;
          return RxAction::FAILED;
        }
        this->sequence_ = want;
        const uint16_t missing = static_cast<uint16_t>(this->expected_ - this->size_);
        const uint16_t take = info.payload_len < missing ? info.payload_len : missing;
        std::memcpy(this->buffer_ + this->size_, &data[info.payload_offset], take);
        this->size_ = static_cast<uint16_t>(this->size_ + take);
        this->timer_start_ = now_ms;
        if (this->size_ >= this->expected_) {
          this->state_ = State::IDLE;
          return RxAction::MESSAGE_COMPLETE;
        }
        // block_size 0 would mean "no further flow control", which we never grant (B7), so a
        // zero here can only come from a block we already exhausted.
        if (this->block_remaining_ != 0) {
          this->block_remaining_--;
          if (this->block_remaining_ == 0) {
            this->block_remaining_ = cfg.block_size;
            encode_flow_control(cfg, FlowStatus::CONTINUE, cfg.block_size, cfg.st_min_raw, out);
            return RxAction::SEND_FLOW_CONTROL;
          }
        }
        return RxAction::NOTHING;
      }

      case PciType::FLOW_CONTROL:
        // Belongs to the send machine; the component routes it there.
        return RxAction::NOTHING;

      case PciType::INVALID:
      default:
        this->ignored_++;
        return RxAction::NOTHING;
    }
  }

  /// Called once per loop iteration; only ever produces a timeout.
  RxAction poll(const IsoTpConfig &cfg, uint32_t now_ms) {
    if (this->state_ != State::RECEIVING)
      return RxAction::NOTHING;
    if (elapsed_ms(now_ms, this->timer_start_) >= cfg.n_cr_timeout_ms) {
      this->state_ = State::IDLE;
      this->error_ = IsoTpError::TIMEOUT_CF;
      return RxAction::FAILED;
    }
    return RxAction::NOTHING;
  }

 protected:
  enum class State : uint8_t { IDLE, RECEIVING };

  uint8_t *buffer_{nullptr};
  uint16_t capacity_{0};
  uint16_t size_{0};
  uint16_t expected_{0};
  uint32_t timer_start_{0};
  uint32_t ignored_{0};
  State state_{State::IDLE};
  IsoTpError error_{IsoTpError::NONE};
  uint8_t sequence_{0};
  uint8_t block_remaining_{0};
};

}  // namespace esphome::isotp
