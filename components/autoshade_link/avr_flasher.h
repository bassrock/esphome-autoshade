#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#include <string>

namespace esphome {
namespace autoshade_link {

/// Reflashes the board's ATmega328P through its Optiboot bootloader, over the
/// same serial link the protocol uses. The caller resets the chip (DTR pulse
/// through the '16U2) and then hands every received byte to feed() until
/// done() is true.
///
/// STK500v1, the subset Optiboot speaks: GET_SYNC, ENTER_PROGMODE, then per
/// 128-byte page LOAD_ADDRESS + PROG_PAGE, then LOAD_ADDRESS + READ_PAGE for
/// each page to verify, then LEAVE_PROGMODE (Optiboot then starts the sketch).
/// Optiboot's watchdog restarts the sketch after ~1 s without a command, so a
/// failure part-way leaves the old or a partial sketch, never a dead
/// bootloader; retrying, or a laptop with Arduino IDE, recovers it.
class AvrFlasher {
 public:
  void set_image(const uint8_t *image, size_t len) {
    this->image_ = image;
    this->len_ = len;
  }
  bool has_image() const { return this->image_ != nullptr && this->len_ > 0; }

  /// Call right after the reset pulse.
  void start(uart::UARTDevice *uart);
  /// Run from loop(): timeouts and sending the next command.
  void loop();
  void feed(uint8_t c);

  bool busy() const { return this->state_ != STATE_IDLE && this->state_ != STATE_DONE && this->state_ != STATE_FAILED; }
  bool done() const { return this->state_ == STATE_DONE || this->state_ == STATE_FAILED; }
  bool succeeded() const { return this->state_ == STATE_DONE; }
  const std::string &status() const { return this->status_; }

 protected:
  enum State : uint8_t {
    STATE_IDLE,
    STATE_SYNC,
    STATE_SETTLE,
    STATE_PROGMODE,
    STATE_WRITE_ADDR,
    STATE_WRITE_PAGE,
    STATE_VERIFY_ADDR,
    STATE_VERIFY_PAGE,
    STATE_LEAVE,
    STATE_DONE,
    STATE_FAILED,
  };

  void send_(const uint8_t *data, size_t len, size_t reply_len);
  void next_();
  void fail_(const char *why);
  void set_status_(const char *fmt, ...);
  size_t pages_() const { return (this->len_ + PAGE - 1) / PAGE; }

  static const size_t PAGE = 128;

  uart::UARTDevice *uart_{nullptr};
  const uint8_t *image_{nullptr};
  size_t len_{0};

  State state_{STATE_IDLE};
  size_t page_{0};
  uint32_t started_ms_{0};
  uint32_t sent_ms_{0};
  bool waiting_{false};
  uint8_t reply_[PAGE + 2];
  size_t reply_len_{0};
  size_t reply_want_{0};
  std::string status_{"idle"};
};

}  // namespace autoshade_link
}  // namespace esphome
