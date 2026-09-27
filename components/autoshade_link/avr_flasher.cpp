#include "avr_flasher.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace esphome {
namespace autoshade_link {

static const char *const TAG = "autoshade_link.flash";

// STK500v1
static const uint8_t STK_OK = 0x10;
static const uint8_t STK_INSYNC = 0x14;
static const uint8_t CRC_EOP = 0x20;
static const uint8_t STK_GET_SYNC = 0x30;
static const uint8_t STK_ENTER_PROGMODE = 0x50;
static const uint8_t STK_LEAVE_PROGMODE = 0x51;
static const uint8_t STK_LOAD_ADDRESS = 0x55;
static const uint8_t STK_PROG_PAGE = 0x64;
static const uint8_t STK_READ_PAGE = 0x74;

static const uint32_t SYNC_WINDOW_MS = 2500;
// This board's Optiboot (4.x) blinks the LED 3 times (~375 ms) after reset
// before it reads the UART. Its receive buffer holds 2 bytes, so anything
// more sent during the blink overruns, and a GET_SYNC that loses its 0x20
// makes verifySpace() bail straight to the sketch. So: say nothing until the
// blink is over, then send one GET_SYNC at a time.
static const uint32_t SYNC_FIRST_MS = 500;
static const uint32_t SYNC_RETRY_MS = 250;  // it answers at once once listening
static const uint32_t REPLY_TIMEOUT_MS = 500;
static const uint32_t SETTLE_MS = 100;  // well inside Optiboot's ~1 s watchdog

void AvrFlasher::start(uart::UARTDevice *uart) {
  this->uart_ = uart;
  this->page_ = 0;
  this->started_ms_ = millis();
  this->waiting_ = false;
  this->reply_len_ = 0;
  this->trace_len_ = 0;
  this->state_ = STATE_SYNC;
  this->set_status_("syncing with bootloader");
}

void AvrFlasher::send_(const uint8_t *data, size_t len, size_t reply_len) {
  this->reply_len_ = 0;
  this->reply_want_ = reply_len;
  this->waiting_ = true;
  this->sent_ms_ = millis();
  this->uart_->write_array(data, len);
}

void AvrFlasher::loop() {
  if (!this->busy())
    return;
  const uint32_t now = millis();

  if (this->state_ == STATE_SYNC) {
    if (now - this->started_ms_ > SYNC_WINDOW_MS) {
      this->fail_("bootloader did not answer (no Optiboot, or reset failed)");
      return;
    }
    if (now - this->started_ms_ < SYNC_FIRST_MS)
      return;
    // Not while half a reply is in: resending would throw its tail away.
    if (!this->waiting_ || (this->reply_len_ == 0 && now - this->sent_ms_ >= SYNC_RETRY_MS)) {
      const uint8_t cmd[] = {STK_GET_SYNC, CRC_EOP};
      this->send_(cmd, sizeof(cmd), 2);
    }
    return;
  }

  if (this->state_ == STATE_SETTLE) {
    if (now - this->sent_ms_ >= SETTLE_MS) {
      // One more GET_SYNC, now with nothing else in flight, checked strictly.
      // A stray reply from a retry would fail here, not mid-programming.
      this->state_ = STATE_CONFIRM;
      const uint8_t cmd[] = {STK_GET_SYNC, CRC_EOP};
      this->send_(cmd, sizeof(cmd), 2);
    }
    return;
  }

  if (this->waiting_ && now - this->sent_ms_ > REPLY_TIMEOUT_MS) {
    char why[48];
    snprintf(why, sizeof(why), "no reply at page %u", (unsigned) this->page_);
    this->fail_(why);
    return;
  }
  if (!this->waiting_)
    this->next_();
}

void AvrFlasher::feed(uint8_t c) {
  if (!this->busy())
    return;
  if (this->trace_len_ < sizeof(this->trace_))
    this->trace_[this->trace_len_++] = c;
  if (!this->waiting_)
    return;

  if (this->state_ == STATE_SYNC) {
    // Leftover protocol text may still be arriving; look for INSYNC OK.
    if (this->reply_len_ == 0 && c != STK_INSYNC)
      return;
    this->reply_[this->reply_len_++] = c;
    if (this->reply_len_ < 2)
      return;
    this->waiting_ = false;
    if (this->reply_[1] != STK_OK) {
      this->reply_len_ = 0;
      this->waiting_ = true;  // keep listening until the retry timer resends
      return;
    }
    // Earlier GET_SYNC retries may still be answered. Let those drain before
    // the next command, or every reply after would be read off by two bytes.
    this->state_ = STATE_SETTLE;
    this->sent_ms_ = millis();
    return;
  }

  if (this->reply_len_ < sizeof(this->reply_))
    this->reply_[this->reply_len_] = c;
  this->reply_len_++;
  if (this->reply_len_ < this->reply_want_)
    return;

  this->waiting_ = false;
  if (this->reply_[0] != STK_INSYNC || this->reply_[this->reply_want_ - 1] != STK_OK) {
    char why[64];
    if (this->reply_[0] == 'V')
      snprintf(why, sizeof(why), "bootloader exited (sketch banner) at step %u page %u", this->state_,
               (unsigned) this->page_);
    else
      snprintf(why, sizeof(why), "bad reply 0x%02X at step %u page %u", this->reply_[0], this->state_,
               (unsigned) this->page_);
    this->fail_(why);
    return;
  }

  if (this->state_ == STATE_VERIFY_PAGE) {
    const size_t base = this->page_ * PAGE;
    for (size_t i = 0; i < PAGE; i++) {
      const uint8_t want = base + i < this->len_ ? this->image_[base + i] : 0xFF;
      if (this->reply_[1 + i] != want) {
        char why[48];
        snprintf(why, sizeof(why), "verify mismatch at 0x%04X", (unsigned) (base + i));
        this->fail_(why);
        return;
      }
    }
  }
  // next_() is called from loop(), so one command goes out per loop pass.
}

// Called with no reply outstanding: the last command succeeded, so send the
// one after it.
void AvrFlasher::next_() {
  switch (this->state_) {
    case STATE_CONFIRM:
      this->state_ = STATE_PROGMODE;
      break;
    case STATE_PROGMODE:
      this->page_ = 0;
      this->state_ = STATE_WRITE_ADDR;
      break;
    case STATE_WRITE_ADDR:
      this->state_ = STATE_WRITE_PAGE;
      break;
    case STATE_WRITE_PAGE:
      this->page_++;
      if (this->page_ < this->pages_()) {
        this->state_ = STATE_WRITE_ADDR;
      } else {
        this->page_ = 0;
        this->state_ = STATE_VERIFY_ADDR;
      }
      break;
    case STATE_VERIFY_ADDR:
      this->state_ = STATE_VERIFY_PAGE;
      break;
    case STATE_VERIFY_PAGE:
      this->page_++;
      this->state_ = this->page_ < this->pages_() ? STATE_VERIFY_ADDR : STATE_LEAVE;
      break;
    case STATE_LEAVE:
      this->state_ = STATE_DONE;
      this->set_status_("done: %u bytes written and verified", (unsigned) this->len_);
      ESP_LOGI(TAG, "%s", this->status_.c_str());
      return;
    default:
      return;
  }

  const size_t base = this->page_ * PAGE;
  switch (this->state_) {
    case STATE_PROGMODE: {
      this->set_status_("entering programming mode");
      const uint8_t cmd[] = {STK_ENTER_PROGMODE, CRC_EOP};
      this->send_(cmd, sizeof(cmd), 2);
      break;
    }
    case STATE_WRITE_ADDR:
    case STATE_VERIFY_ADDR: {
      if (this->state_ == STATE_WRITE_ADDR)
        this->set_status_("writing %u%%", (unsigned) (this->page_ * 100 / this->pages_()));
      else
        this->set_status_("verifying %u%%", (unsigned) (this->page_ * 100 / this->pages_()));
      const uint16_t word = base / 2;
      const uint8_t cmd[] = {STK_LOAD_ADDRESS, (uint8_t) (word & 0xFF), (uint8_t) (word >> 8), CRC_EOP};
      this->send_(cmd, sizeof(cmd), 2);
      break;
    }
    case STATE_WRITE_PAGE: {
      uint8_t cmd[4 + PAGE + 1];
      cmd[0] = STK_PROG_PAGE;
      cmd[1] = 0;
      cmd[2] = PAGE;
      cmd[3] = 'F';
      for (size_t i = 0; i < PAGE; i++)
        cmd[4 + i] = base + i < this->len_ ? this->image_[base + i] : 0xFF;
      cmd[4 + PAGE] = CRC_EOP;
      this->send_(cmd, sizeof(cmd), 2);
      break;
    }
    case STATE_VERIFY_PAGE: {
      const uint8_t cmd[] = {STK_READ_PAGE, 0, PAGE, 'F', CRC_EOP};
      this->send_(cmd, sizeof(cmd), PAGE + 2);
      break;
    }
    case STATE_LEAVE: {
      const uint8_t cmd[] = {STK_LEAVE_PROGMODE, CRC_EOP};
      this->send_(cmd, sizeof(cmd), 2);
      break;
    }
    default:
      break;
  }
}

void AvrFlasher::fail_(const char *why) {
  this->state_ = STATE_FAILED;
  this->waiting_ = false;
  this->set_status_("failed: %s", why);
  ESP_LOGE(TAG, "%s", this->status_.c_str());
  // What the board actually said since the reset, for diagnosis.
  char hex[3 * sizeof(this->trace_) + 1];
  size_t n = 0;
  for (size_t i = 0; i < this->trace_len_; i++)
    n += snprintf(hex + n, sizeof(hex) - n, "%02X ", this->trace_[i]);
  hex[n] = '\0';
  ESP_LOGE(TAG, "first %u bytes received: %s", (unsigned) this->trace_len_, hex);
}

void AvrFlasher::set_status_(const char *fmt, ...) {
  char buf[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  this->status_ = buf;
}

}  // namespace autoshade_link
}  // namespace esphome
