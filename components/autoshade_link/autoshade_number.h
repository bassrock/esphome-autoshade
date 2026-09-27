#pragma once

#include <cmath>
#include "esphome/components/number/number.h"
#include "autoshade_cover.h"

namespace esphome {
namespace autoshade_link {

/// Shade length in inches. The cover owns and persists the value; this just
/// forwards edits from HA and shows what the cover currently uses.
class AutoShadeLengthNumber : public number::Number {
 public:
  void set_cover(AutoShadeCover *c, float steps_per_inch) {
    this->cover_ = c;
    this->steps_per_inch_ = steps_per_inch;
  }

 protected:
  void control(float value) override {
    if (this->cover_ != nullptr)
      this->cover_->set_length_steps((int32_t) lroundf(value * this->steps_per_inch_));
  }

  AutoShadeCover *cover_{nullptr};
  float steps_per_inch_{177.0f};
};

}  // namespace autoshade_link
}  // namespace esphome
