#include "autoshade_link.h"
#include "autoshade_cover.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace esphome {
namespace autoshade_link {

static const char *const TAG = "autoshade_link";

void AutoShadeLink::setup() { this->send_line_("V"); }

void AutoShadeLink::register_cover(AutoShadeCover *cover) { this->covers_.push_back(cover); }

void AutoShadeLink::register_button(uint8_t bit, binary_sensor::BinarySensor *sensor) {
  if (bit < 5)
    this->buttons_[bit] = sensor;
}

void AutoShadeLink::dump_config() {
  ESP_LOGCONFIG(TAG, "AUTOSHADE link:");
  ESP_LOGCONFIG(TAG, "  Covers: %u", (unsigned) this->covers_.size());
  for (auto *c : this->covers_)
    ESP_LOGCONFIG(TAG, "    motor %u '%s': %d steps travel", c->get_motor(),
                  c->get_name().c_str(), c->get_travel_steps());
}

void AutoShadeLink::send_line_(const char *line) {
  if (this->updating_) {
    ESP_LOGW(TAG, "board firmware update in progress; dropped '%s'", line);
    return;
  }
  // Status polls must not pile up behind a stalled link.
  if (line[0] == 'Q' && line[1] == '\0' && this->tx_queue_.size() >= 4)
    return;
  this->tx_queue_.emplace_back(line);
  this->pump_tx_();
}

void AutoShadeLink::pump_tx_() {
  const uint32_t now = millis();
  if (this->awaiting_reply_ && (now - this->sent_ms_) < 300)
    return;  // still waiting for the last reply
  if (this->tx_queue_.empty())
    return;
  const std::string &l = this->tx_queue_.front();
  this->write_str(l.c_str());
  this->write_byte('\n');
  this->tx_queue_.pop_front();
  this->awaiting_reply_ = true;
  this->sent_ms_ = now;
}

void AutoShadeLink::send_move(uint8_t motor, int32_t target, uint16_t steps_per_sec,
                              uint16_t accel, uint8_t end_mask) {
  char out[64];
  snprintf(out, sizeof(out), "T %u %ld %u %u %u", motor, (long) target, steps_per_sec, accel,
           end_mask);
  ESP_LOGD(TAG, "-> %s", out);
  this->last_move_ms_ = millis();
  this->send_line_(out);
}

void AutoShadeLink::send_stop(uint8_t motor) {
  char out[16];
  snprintf(out, sizeof(out), "S %u", motor);
  ESP_LOGD(TAG, "-> %s", out);
  this->send_line_(out);
}

void AutoShadeLink::send_set_position(uint8_t motor, int32_t pos) {
  char out[32];
  snprintf(out, sizeof(out), "X %u %ld", motor, (long) pos);
  ESP_LOGD(TAG, "-> %s", out);
  this->send_line_(out);
}

void AutoShadeLink::send_coils(uint8_t motor, uint8_t mask) {
  char out[24];
  snprintf(out, sizeof(out), "C %u %u", motor, mask & 0x0F);
  this->send_line_(out);
}

void AutoShadeLink::send_lcd(uint8_t row, const std::string &text) {
  row = row ? 1 : 0;
  this->pending_lcd_[row] = text;
  this->pending_lcd_set_[row] = true;
  if (this->motion_quiet_())
    this->flush_display_();
}

void AutoShadeLink::send_backlight(uint8_t bits) {
  this->pending_backlight_ = bits & 0x07;
  this->pending_backlight_set_ = true;
  if (this->motion_quiet_())
    this->flush_display_();
}

bool AutoShadeLink::motion_quiet_() const {
  return !this->board_moving_ && millis() - this->last_move_ms_ >= 1000;
}

void AutoShadeLink::flush_display_() {
  char out[40];
  for (uint8_t row = 0; row < 2; row++) {
    if (!this->pending_lcd_set_[row])
      continue;
    snprintf(out, sizeof(out), "L %u %.16s", row, this->pending_lcd_[row].c_str());
    this->send_line_(out);
    this->pending_lcd_set_[row] = false;
  }
  if (this->pending_backlight_set_) {
    snprintf(out, sizeof(out), "G %u", this->pending_backlight_);
    this->send_line_(out);
    this->pending_backlight_set_ = false;
  }
}

void AutoShadeLink::set_all_speed(uint16_t sps) {
  for (auto *c : this->covers_)
    c->set_max_speed(sps);
}

void AutoShadeLink::set_all_accel(uint16_t acc) {
  for (auto *c : this->covers_)
    c->set_acceleration(acc);
}

void AutoShadeLink::update() {
  if (!this->updating_)
    this->send_line_("Q");
}

// ---- board firmware update ----

bool AutoShadeLink::start_avr_update() {
#ifdef AUTOSHADE_LINK_USB
  if (this->updating_)
    return false;
  if (!this->flasher_.has_image()) {
    ESP_LOGW(TAG, "no board firmware bundled; cannot update");
    return false;
  }
  if (this->usb_channel_ == nullptr) {
    ESP_LOGW(TAG, "board firmware update needs usb_channel: set on autoshade_link");
    return false;
  }
  for (auto *c : this->covers_) {
    if (c->is_moving()) {
      ESP_LOGW(TAG, "'%s' is moving; board firmware update refused", c->get_name().c_str());
      return false;
    }
  }
  ESP_LOGI(TAG, "updating board firmware %s -> %s", this->board_version_.empty() ? "?" : this->board_version_.c_str(),
           this->bundled_version_.c_str());
  this->tx_queue_.clear();
  this->awaiting_reply_ = false;
  this->updating_ = true;
  this->update_step_ = 0;
  this->update_ms_ = millis();
  this->set_dtr_(false);
  return true;
#else
  ESP_LOGW(TAG, "board firmware update needs usb_channel: set on autoshade_link");
  return false;
#endif
}

#ifdef AUTOSHADE_LINK_USB
// The '16U2 holds the '328P's RESET low while DTR is asserted, through a
// capacitor, so asserting DTR after a moment deasserted is one reset pulse.
void AutoShadeLink::set_dtr_(bool on) {
  static const uint8_t CDC_SET_CONTROL_LINE_STATE = 0x22;
  this->usb_channel_->get_parent()->control_transfer(
      usb_host::USB_TYPE_CLASS | usb_host::USB_RECIP_INTERFACE, CDC_SET_CONTROL_LINE_STATE, on ? 0x0003 : 0x0000,
      0, [on](const usb_host::TransferStatus &status) {
        if (!status.success)
          ESP_LOGW(TAG, "DTR %s failed: %X", on ? "on" : "off", status.error_code);
      });
}
#endif

void AutoShadeLink::finish_avr_update_() {
  this->updating_ = false;
  this->len_ = 0;
  this->seen_status_ = false;  // the board restarts; decide positions afresh
  this->lcd_redraw_ = true;
  this->last_rx_ms_ = millis();
  this->board_version_.clear();
  if (this->flasher_.succeeded())
    ESP_LOGI(TAG, "board firmware updated; waiting for it to start");
  else
    ESP_LOGE(TAG, "board firmware update failed: %s", this->flasher_.status().c_str());
  this->send_line_("V");
}

void AutoShadeLink::loop() {
  if (this->updating_) {
    const uint32_t now = millis();
    while (this->available()) {
      uint8_t c;
      if (!this->read_byte(&c))
        break;
      if (this->update_step_ == 2)
        this->flasher_.feed(c);  // anything before that is the old sketch talking
    }
#ifdef AUTOSHADE_LINK_USB
    if (this->update_step_ == 0 && now - this->update_ms_ >= 100) {
      this->set_dtr_(true);
      this->update_step_ = 1;
      this->update_ms_ = now;
    } else if (this->update_step_ == 1 && now - this->update_ms_ >= 20) {
      this->update_step_ = 2;
      this->flasher_.start(this);
    }
#endif
    if (this->update_step_ == 2) {
      this->flasher_.loop();
      if (this->flasher_.done())
        this->finish_avr_update_();
    }
    return;
  }

  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c))
      break;
    this->last_rx_ms_ = millis();
    if (c == '\r')
      continue;
    if (c == '\n') {
      this->buf_[this->len_] = '\0';
      if (this->len_ > 0)
        this->handle_line_(this->buf_);
      this->len_ = 0;
    } else if (this->len_ < sizeof(this->buf_) - 1) {
      this->buf_[this->len_++] = (char) c;
    } else {
      this->len_ = 0;
    }
  }

  if ((this->pending_lcd_set_[0] || this->pending_lcd_set_[1] || this->pending_backlight_set_) &&
      this->motion_quiet_())
    this->flush_display_();

  this->pump_tx_();

  // No traffic for 5 s means the board is gone (unplugged, or USB dropped).
  if (this->linked_ && (millis() - this->last_rx_ms_) > 5000) {
    this->linked_ = false;
    ESP_LOGW(TAG, "link to board lost");
  }
}

void AutoShadeLink::handle_line_(char *line) {
  this->awaiting_reply_ = false;  // every command gets exactly one line back
  switch (line[0]) {
    case 'Q':
      this->handle_status_(line);
      break;
    case 'V':
      ESP_LOGI(TAG, "board: %s", line);
      // It just announced itself, so it almost certainly just reset.
      this->seen_status_ = false;
      {
        // V AUTOSHADE-DUMB 3.3 N=6
        const char *sp = strchr(line + 2, ' ');
        if (sp != nullptr) {
          const char *end = strchr(sp + 1, ' ');
          this->board_version_.assign(sp + 1, end != nullptr ? end - sp - 1 : strlen(sp + 1));
        }
      }
      break;
    case 'O':  // OK
      break;
    case 'E':  // ERR ...
      ESP_LOGW(TAG, "board: %s", line);
      break;
    case 'R':  // raw EEPROM bytes
      ESP_LOGI(TAG, "board EEPROM: %s", line + 1);
      break;
    default:
      ESP_LOGV(TAG, "board: %s", line);
      break;
  }
}

// Q P=p1,..,p6 M=<moving> K=<buttons> U=<uptime_s> A=<adc0> B=<adc1> [E=<0|1> F=<0|1>]
// E= and F= arrive from board firmware 3.2 on; older boards omit them.
void AutoShadeLink::handle_status_(char *line) {
  int32_t pos[6] = {0, 0, 0, 0, 0, 0};
  uint32_t moving = 0, keys = 0, uptime = 0;
  int32_t adc0 = -1, adc1 = -1;
  int32_t eeprom = -1, fault = 0;  // -1: firmware older than 3.2
  bool have_pos = false;

  char *save = nullptr;
  for (char *tok = strtok_r(line, " ", &save); tok != nullptr;
       tok = strtok_r(nullptr, " ", &save)) {
    if (strncmp(tok, "P=", 2) == 0) {
      char *p = tok + 2;
      for (uint8_t i = 0; i < 6 && p != nullptr && *p != '\0'; i++) {
        pos[i] = (int32_t) strtol(p, nullptr, 10);
        char *comma = strchr(p, ',');
        p = (comma == nullptr) ? nullptr : comma + 1;
      }
      have_pos = true;
    } else if (strncmp(tok, "M=", 2) == 0) {
      moving = (uint32_t) strtoul(tok + 2, nullptr, 10);
    } else if (strncmp(tok, "K=", 2) == 0) {
      keys = (uint32_t) strtoul(tok + 2, nullptr, 10);
    } else if (strncmp(tok, "U=", 2) == 0) {
      uptime = (uint32_t) strtoul(tok + 2, nullptr, 10);
    } else if (strncmp(tok, "A=", 2) == 0) {
      adc0 = (int32_t) strtol(tok + 2, nullptr, 10);
    } else if (strncmp(tok, "B=", 2) == 0) {
      adc1 = (int32_t) strtol(tok + 2, nullptr, 10);
    } else if (strncmp(tok, "E=", 2) == 0) {
      eeprom = (int32_t) strtol(tok + 2, nullptr, 10);
    } else if (strncmp(tok, "F=", 2) == 0) {
      fault = (int32_t) strtol(tok + 2, nullptr, 10);
    }
  }

  if (!have_pos)
    return;

  this->linked_ = true;
  this->board_moving_ = moving != 0;

  // First status since we booted, or the board's uptime went backwards (it
  // reset under us). Decide who has the better copy of the positions.
  if (!this->seen_status_ || uptime < this->last_uptime_) {
    const bool reset_under_us = this->esp_synced_;
    this->last_uptime_ = uptime;
    this->seen_status_ = true;
    this->esp_synced_ = true;
    this->lcd_redraw_ = true;  // a reset board shows its boot banner, not our text

    // The board saves to EEPROM every time a motor comes to rest, so its copy
    // is exact — unless it reset part-way through a move, in which case its
    // EEPROM holds the pre-move position and our last poll is closer.
    if (eeprom == 1 && !(reset_under_us && this->last_moving_any_)) {
      ESP_LOGI(TAG, "positions read from board (uptime %u)", uptime);
      for (auto *c : this->covers_) {
        const uint8_t m = c->get_motor();
        if (m >= 1 && m <= 6)
          c->adopt(pos[m - 1]);
      }
      this->last_moving_any_ = moving != 0;
      return;
    }

    if (eeprom == 1)
      ESP_LOGW(TAG, "board reset mid-move; pushing last known positions (re-home recommended)");
    else if (eeprom == 0)
      ESP_LOGW(TAG, "board has no saved positions; pushing ours");
    else
      ESP_LOGW(TAG, "board firmware predates 3.2 (no position storage); pushing ours");
    this->resync_positions_();
    return;  // its P= values are stale; skip this round
  }
  this->last_uptime_ = uptime;
  this->last_moving_any_ = moving != 0;

  if (fault && !this->last_fault_)
    ESP_LOGW(TAG, "12 V dropped out: board cancelled motion (positions still exact)");
  this->last_fault_ = fault != 0;

  for (auto *c : this->covers_) {
    const uint8_t m = c->get_motor();
    if (m >= 1 && m <= 6)
      c->update_from_link(pos[m - 1], (moving >> (m - 1)) & 0x01);
  }

  for (uint8_t b = 0; b < 5; b++) {
    if (this->buttons_[b] != nullptr)
      this->buttons_[b]->publish_state((keys >> b) & 0x01);
  }

  if (this->supply_ != nullptr && adc0 >= 0)
    this->supply_->publish_state(adc0 * this->supply_scale_);
  if (this->battery_ != nullptr && adc1 >= 0)
    this->battery_->publish_state(adc1 * this->battery_scale_);
}

void AutoShadeLink::resync_positions_() {
  for (auto *c : this->covers_)
    c->resync();
}

}  // namespace autoshade_link
}  // namespace esphome
