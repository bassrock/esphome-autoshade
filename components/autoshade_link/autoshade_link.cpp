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
  this->write_str(line);
  this->write_byte('\n');
}

void AutoShadeLink::send_move(uint8_t motor, int32_t target, uint16_t steps_per_sec,
                              uint16_t accel, uint8_t end_mask) {
  char out[64];
  snprintf(out, sizeof(out), "T %u %ld %u %u %u", motor, (long) target, steps_per_sec, accel,
           end_mask);
  ESP_LOGD(TAG, "-> %s", out);
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
  char out[40];
  snprintf(out, sizeof(out), "L %u %.16s", row ? 1 : 0, text.c_str());
  this->send_line_(out);
}

void AutoShadeLink::send_backlight(uint8_t bits) {
  char out[16];
  snprintf(out, sizeof(out), "G %u", bits & 0x07);
  this->send_line_(out);
}

void AutoShadeLink::update() { this->send_line_("Q"); }

void AutoShadeLink::loop() {
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

  // No traffic for 5 s means the board is gone (unplugged, or USB dropped).
  if (this->linked_ && (millis() - this->last_rx_ms_) > 5000) {
    this->linked_ = false;
    ESP_LOGW(TAG, "link to board lost");
  }
}

void AutoShadeLink::handle_line_(char *line) {
  switch (line[0]) {
    case 'Q':
      this->handle_status_(line);
      break;
    case 'V':
      ESP_LOGI(TAG, "board: %s", line);
      // It just announced itself, so it almost certainly just reset.
      this->seen_status_ = false;
      break;
    case 'O':  // OK
      break;
    case 'E':  // ERR ...
      ESP_LOGW(TAG, "board: %s", line);
      break;
    default:
      ESP_LOGV(TAG, "board: %s", line);
      break;
  }
}

// Q P=p1,..,p6 M=<moving> K=<buttons> U=<uptime_s> A=<adc0> B=<adc1>
void AutoShadeLink::handle_status_(char *line) {
  int32_t pos[6] = {0, 0, 0, 0, 0, 0};
  uint32_t moving = 0, keys = 0, uptime = 0;
  int32_t adc0 = -1, adc1 = -1;
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
    }
  }

  if (!have_pos)
    return;

  this->linked_ = true;

  // Uptime going backwards, or a first sighting, means the board's step
  // counters are not ours. Opening the USB port pulses DTR, which resets the
  // AVR, so this is expected on every reconnect rather than exceptional.
  if (!this->seen_status_ || uptime < this->last_uptime_) {
    ESP_LOGW(TAG, "board reset (uptime %u); pushing our positions back", uptime);
    this->last_uptime_ = uptime;
    this->seen_status_ = true;
    this->resync_positions_();
    return;  // its P= values are stale; skip this round
  }
  this->last_uptime_ = uptime;

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
