#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

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

  void set_supply_sensor(sensor::Sensor *s) { this->supply_ = s; }
  void set_battery_sensor(sensor::Sensor *s) { this->battery_ = s; }
  /// ADC counts to volts. Defaults come from the original firmware's comments.
  void set_supply_scale(float v) { this->supply_scale_ = v; }
  void set_battery_scale(float v) { this->battery_scale_ = v; }

  bool is_linked() const { return this->linked_; }

 protected:
  void send_line_(const char *line);
  void handle_line_(char *line);
  void handle_status_(char *line);
  void resync_positions_();

  std::vector<AutoShadeCover *> covers_;
  binary_sensor::BinarySensor *buttons_[5]{nullptr, nullptr, nullptr, nullptr, nullptr};

  char buf_[192];
  uint16_t len_{0};
  uint32_t last_uptime_{0};
  bool seen_status_{false};
  bool linked_{false};
  uint32_t last_rx_ms_{0};

  sensor::Sensor *supply_{nullptr};
  sensor::Sensor *battery_{nullptr};
  float supply_scale_{0.01238f};
  float battery_scale_{0.009872f};
};

}  // namespace autoshade_link
}  // namespace esphome
