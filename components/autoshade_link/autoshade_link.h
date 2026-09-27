#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#ifdef AUTOSHADE_LINK_USB
#include "esphome/components/usb_uart/usb_uart.h"
#endif
#include "avr_flasher.h"

#include <deque>
#include <string>
#include <vector>

namespace esphome {
namespace autoshade_link {

class AutoShadeCover;

static const uint8_t COILS_COAST = 0x00;
static const uint8_t COILS_BRAKE = 0x0F;

/// Talks to the AUTOSHADE board's ATmega328P running autoshade_dumb.ino.
///
/// The board holds no policy at all: it executes step trains, counts steps,
/// reports raw button bits and raw ADC counts, and writes literal text to the
/// LCD. Everything that decides anything lives on this side.
class AutoShadeLink : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  void register_cover(AutoShadeCover *cover);
  void register_button(uint8_t bit, binary_sensor::BinarySensor *sensor);

  // Motor numbers are 1-6, matching the board's silkscreen.
  void send_move(uint8_t motor, int32_t target, uint16_t steps_per_sec, uint16_t accel,
                 uint8_t end_mask);
  void send_stop(uint8_t motor);
  void send_set_position(uint8_t motor, int32_t pos);
  void send_coils(uint8_t motor, uint8_t mask);
  void send_lcd(uint8_t row, const std::string &text);
  void send_backlight(uint8_t bits);
  /// Any protocol line, verbatim. Used for R (EEPROM dump) from YAML.
  void send_raw(const std::string &line) { this->send_line_(line.c_str()); }

  void set_supply_sensor(sensor::Sensor *s) { this->supply_ = s; }
  void set_battery_sensor(sensor::Sensor *s) { this->battery_ = s; }
  /// ADC counts to volts. Defaults come from the original firmware's comments.
  void set_supply_scale(float v) { this->supply_scale_ = v; }
  void set_battery_scale(float v) { this->battery_scale_ = v; }

  bool is_linked() const { return this->linked_; }

  /// Speed / acceleration for every cover, from the next move on. Full steps.
  void set_all_speed(uint16_t sps);
  void set_all_accel(uint16_t acc);

#ifdef AUTOSHADE_LINK_USB
  /// The usb_uart channel the board is on. Needed to pulse DTR (the '328P's
  /// reset) for a firmware update.
  void set_usb_channel(usb_uart::USBUartChannel *channel) { this->usb_channel_ = channel; }
#endif
  /// The autoshade_dumb.ino image compiled into this ESP firmware.
  void set_avr_image(const uint8_t *image, size_t len, const char *version) {
    this->flasher_.set_image(image, len);
    this->bundled_version_ = version;
  }
  /// Reflash the board with the bundled image. Refused while a shade moves.
  bool start_avr_update();
  bool is_updating() const { return this->updating_; }
  /// Version from the board's last V banner, e.g. "3.3"; empty until seen.
  const std::string &board_version() const { return this->board_version_; }
  const std::string &bundled_version() const { return this->bundled_version_; }
  const std::string &avr_update_status() const { return this->flasher_.status(); }
  /// Covers in YAML order; used by the setup buttons' "which shade" select.
  AutoShadeCover *cover_at(size_t i) { return i < this->covers_.size() ? this->covers_[i] : nullptr; }

  /// True once after the board resets, and once a minute otherwise. The LCD
  /// lambda uses it to forget what it last sent and redraw everything, so a
  /// line that got mangled in transit never sticks on the screen.
  bool take_lcd_redraw() {
    const uint32_t now = millis();
    if (this->lcd_redraw_ || (now - this->last_redraw_ms_) > 60000) {
      this->lcd_redraw_ = false;
      this->last_redraw_ms_ = now;
      return true;
    }
    return false;
  }

 protected:
  void send_line_(const char *line);
  void pump_tx_();
  void handle_line_(char *line);
  void handle_status_(char *line);
  void resync_positions_();
  void finish_avr_update_();
#ifdef AUTOSHADE_LINK_USB
  void set_dtr_(bool on);
  usb_uart::USBUartChannel *usb_channel_{nullptr};
#endif

  // The board writes the LCD by bit-banging the MCP23017 (~35 ms a line),
  // which freezes its step train, and a loaded stepper cannot restart at speed
  // after that, so it stalls and grinds. LCD and backlight writes are held
  // while any motor runs (and for 1 s after a move is sent), then flushed.
  bool motion_quiet_() const;
  void flush_display_();
  bool board_moving_{false};
  uint32_t last_move_ms_{0};
  std::string pending_lcd_[2];
  bool pending_lcd_set_[2]{false, false};
  uint8_t pending_backlight_{0};
  bool pending_backlight_set_{false};

  AvrFlasher flasher_;
  bool updating_{false};
  uint8_t update_step_{0};  // 0 DTR off sent, 1 DTR on sent, 2 flashing
  uint32_t update_ms_{0};
  std::string board_version_;
  std::string bundled_version_;

  std::vector<AutoShadeCover *> covers_;
  binary_sensor::BinarySensor *buttons_[5]{nullptr, nullptr, nullptr, nullptr, nullptr};

  char buf_[192];
  uint16_t len_{0};
  uint32_t last_uptime_{0};
  bool seen_status_{false};
  bool linked_{false};
  uint32_t last_rx_ms_{0};
  bool lcd_redraw_{true};
  bool esp_synced_{false};       // positions agreed with the board since our boot
  bool last_moving_any_{false};  // was anything moving at the last status?
  bool last_fault_{false};
  uint32_t last_redraw_ms_{0};

  // One command in flight at a time. The AVR's serial buffer is 64 bytes and an
  // LCD line takes it ~25 ms to write over I2C, so back-to-back lines overflow
  // it and arrive mangled. Waiting for each reply makes that impossible.
  std::deque<std::string> tx_queue_;
  bool awaiting_reply_{false};
  uint32_t sent_ms_{0};

  sensor::Sensor *supply_{nullptr};
  sensor::Sensor *battery_{nullptr};
  float supply_scale_{0.01238f};
  float battery_scale_{0.009872f};
};

}  // namespace autoshade_link
}  // namespace esphome
