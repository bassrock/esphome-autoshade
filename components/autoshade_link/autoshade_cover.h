#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/cover/cover.h"
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
  void set_travel_steps(int32_t steps) { this->travel_steps_ = steps; }
  void set_max_speed(uint16_t sps) { this->max_speed_ = sps; }
  void set_acceleration(uint16_t acc) { this->accel_ = acc; }
  void set_hold_mode(HoldMode m) { this->hold_ = m; }
  void set_home_overrun(int32_t steps) { this->home_overrun_ = steps; }

  uint8_t get_motor() const { return this->motor_; }
  int32_t get_travel_steps() const { return this->travel_steps_; }
  int32_t get_stored_steps() const { return this->steps_; }

  /// Drive past the top hard stop at reduced speed, then call that zero.
  void start_home();

  /// Push our remembered position back into the board after it has reset.
  void resync();

  /// Called by the link on every status poll.
  void update_from_link(int32_t steps, bool moving);

 protected:
  void publish_(int32_t steps, cover::CoverOperation op);
  uint8_t end_mask_() const { return this->hold_ == HOLD_BRAKE ? COILS_BRAKE : COILS_COAST; }

  AutoShadeLink *parent_{nullptr};
  uint8_t motor_{1};
  int32_t travel_steps_{15000};
  uint16_t max_speed_{400};
  uint16_t accel_{400};
  HoldMode hold_{HOLD_BRAKE};
  int32_t home_overrun_{0};

  int32_t steps_{0};
  bool was_moving_{false};
  bool homing_{false};
  bool have_steps_{false};
  ESPPreferenceObject pref_;
};

}  // namespace autoshade_link
}  // namespace esphome
