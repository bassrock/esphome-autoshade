#include "autoshade_cover.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome {
namespace autoshade_link {

static const char *const TAG = "autoshade_link.cover";

void AutoShadeCover::setup() {
  // The board keeps no EEPROM, so this is the authoritative position.
  this->pref_ = global_preferences->make_preference<int32_t>(this->get_object_id_hash());
  if (!this->pref_.load(&this->steps_))
    this->steps_ = 0;
  this->publish_(this->steps_, cover::COVER_OPERATION_IDLE);
}

void AutoShadeCover::dump_config() {
  ESP_LOGCONFIG(TAG, "AUTOSHADE cover '%s':", this->get_name().c_str());
  ESP_LOGCONFIG(TAG, "  Motor: %u", this->motor_);
  ESP_LOGCONFIG(TAG, "  Travel: %d steps", this->travel_steps_);
  ESP_LOGCONFIG(TAG, "  Speed: %u steps/s, accel %u", this->max_speed_, this->accel_);
  ESP_LOGCONFIG(TAG, "  Hold: %s", this->hold_ == HOLD_BRAKE ? "brake" : "coast");
  ESP_LOGCONFIG(TAG, "  Home overrun: %d steps", this->home_overrun_);
  ESP_LOGCONFIG(TAG, "  Restored position: %d steps", this->steps_);
}

cover::CoverTraits AutoShadeCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_is_assumed_state(false);
  return traits;
}

void AutoShadeCover::control(const cover::CoverCall &call) {
  if (this->parent_ == nullptr)
    return;

  if (call.get_stop()) {
    this->homing_ = false;
    this->parent_->send_stop(this->motor_);
    return;
  }

  if (call.get_position().has_value()) {
    const float pos = *call.get_position();
    // position 1.0 = fully open = fully up = step 0
    const int32_t target =
        (int32_t) lroundf((1.0f - pos) * (float) this->travel_steps_);
    this->homing_ = false;
    this->parent_->send_move(this->motor_, target, this->max_speed_, this->accel_,
                             this->end_mask_());
  }
}

void AutoShadeCover::start_home() {
  if (this->parent_ == nullptr || this->home_overrun_ <= 0)
    return;
  this->homing_ = true;
  const uint16_t slow = this->max_speed_ / 4 > 20 ? this->max_speed_ / 4 : 20;
  ESP_LOGI(TAG, "'%s': homing into the top stop", this->get_name().c_str());
  this->parent_->send_move(this->motor_, this->steps_ - this->home_overrun_, slow,
                           this->accel_, this->end_mask_());
}

void AutoShadeCover::resync() {
  if (this->parent_ != nullptr)
    this->parent_->send_set_position(this->motor_, this->steps_);
}

void AutoShadeCover::update_from_link(int32_t steps, bool moving) {
  cover::CoverOperation op = cover::COVER_OPERATION_IDLE;
  if (moving) {
    if (this->have_steps_ && steps < this->steps_) {
      op = cover::COVER_OPERATION_OPENING;
    } else if (this->have_steps_ && steps > this->steps_) {
      op = cover::COVER_OPERATION_CLOSING;
    } else {
      op = this->current_operation == cover::COVER_OPERATION_IDLE
               ? cover::COVER_OPERATION_OPENING
               : this->current_operation;
    }
  }

  const bool arrived = this->was_moving_ && !moving;
  this->was_moving_ = moving;
  this->have_steps_ = true;

  if (arrived && this->homing_) {
    // We drove into the hard stop; that place is zero now.
    this->homing_ = false;
    this->steps_ = 0;
    if (this->parent_ != nullptr)
      this->parent_->send_set_position(this->motor_, 0);
    this->pref_.save(&this->steps_);
    this->publish_(0, cover::COVER_OPERATION_IDLE);
    return;
  }

  const bool changed = steps != this->steps_ || op != this->current_operation;
  this->steps_ = steps;

  if (arrived)
    this->pref_.save(&this->steps_);

  if (changed)
    this->publish_(steps, op);
}

void AutoShadeCover::publish_(int32_t steps, cover::CoverOperation op) {
  float pos = 1.0f - ((float) steps / (float) this->travel_steps_);
  this->position = clamp(pos, 0.0f, 1.0f);
  this->current_operation = op;
  this->publish_state();
}

}  // namespace autoshade_link
}  // namespace esphome
