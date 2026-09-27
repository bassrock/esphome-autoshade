#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/hal.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/number/number.h"
#include "autoshade_link.h"

namespace esphome {
namespace autoshade_link {

enum HoldMode : uint8_t {
  HOLD_COAST = 0,  // DRV8871 inputs low, outputs Hi-Z, motor free
  HOLD_BRAKE = 1,  // inputs high, windings shorted, zero current, resists back-drive
};

/// One shade. All of the policy lives here: how far the shade travels, how
/// fast, what it does when it arrives, and where it thinks it is across
/// reboots. The board is told only "run motor n to step N at this rate".
class AutoShadeCover : public cover::Cover, public Component {
 public:
  void setup() override;
  void dump_config() override;
  cover::CoverTraits get_traits() override;
  void control(const cover::CoverCall &call) override;

  void set_parent(AutoShadeLink *parent) { this->parent_ = parent; }
  void set_motor(uint8_t motor) { this->motor_ = motor; }
  /// The YAML value. Only a default: a length set from HA (number entity or
  /// Set Bottom Here) is saved and wins from then on.
  void set_travel_steps(int32_t steps) { this->travel_steps_ = steps; }
  void set_length_number(number::Number *n, float steps_per_inch) {
    this->length_number_ = n;
    this->steps_per_inch_ = steps_per_inch;
  }
  void set_max_speed(uint16_t sps) { this->max_speed_ = sps; }
  void set_acceleration(uint16_t acc) { this->accel_ = acc; }
  void set_hold_mode(HoldMode m) { this->hold_ = m; }
  void set_home_overrun(int32_t steps) { this->home_overrun_ = steps; }

  uint8_t get_motor() const { return this->motor_; }
  int32_t get_travel_steps() const { return this->travel_steps_; }
  int32_t get_stored_steps() const { return this->steps_; }

  /// Drive past the top hard stop at reduced speed, then call that zero.
  void start_home();

  // ---- setup / calibration, used from HA buttons and the length number ----
  /// Move by a relative number of steps (negative = up), ignoring the limits,
  /// so a shade can be driven past where we currently think the ends are.
  void jog(int32_t delta_steps);
  /// "Where the shade is now is fully open." The bottom stays where it was
  /// physically, so the length grows or shrinks by however far we moved.
  bool set_top_here();
  /// "Where the shade is now is fully closed." Length = distance from the top.
  bool set_bottom_here();
  /// "The shade is physically at this position" (1.0 = fully up, 0.0 = fully
  /// down), without moving it. For telling the ESP where a shade is after it
  /// was moved by hand or the counters were lost.
  bool mark_position(float pos);
  /// New full-drop length in steps, from the number entity.
  void set_length_steps(int32_t steps);
  /// Moving, or told to move less than a second ago (the first status poll
  /// that shows the motion can be up to 500 ms behind the command).
  bool is_moving() const { return this->was_moving_ || (millis() - this->last_cmd_ms_) < 1000; }

  /// Push our remembered position back into the board after it has reset.
  void resync();
  /// Take the board's count as the truth (it has EEPROM from firmware 3.2).
  void adopt(int32_t steps);

 protected:
  void home_final_();

 public:

  /// Called by the link on every status poll.
  void update_from_link(int32_t steps, bool moving);

 protected:
  void publish_(int32_t steps, cover::CoverOperation op);
  void save_length_();
  uint8_t end_mask_() const { return this->hold_ == HOLD_BRAKE ? COILS_BRAKE : COILS_COAST; }

  AutoShadeLink *parent_{nullptr};
  uint8_t motor_{1};
  int32_t travel_steps_{15000};
  uint16_t max_speed_{90};
  uint16_t accel_{100};
  HoldMode hold_{HOLD_BRAKE};
  int32_t home_overrun_{0};

  int32_t steps_{0};
  bool was_moving_{false};
  bool homing_{false};
  bool homing_approach_{false};  // stage 1: full speed to where we think zero is
  bool have_steps_{false};
  ESPPreferenceObject pref_;
  ESPPreferenceObject length_pref_;
  uint32_t last_cmd_ms_{0};
  number::Number *length_number_{nullptr};
  float steps_per_inch_{177.0f};
};

}  // namespace autoshade_link
}  // namespace esphome
