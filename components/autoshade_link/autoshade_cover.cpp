#include "autoshade_cover.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome {
namespace autoshade_link {

static const char *const TAG = "autoshade_link.cover";

void AutoShadeCover::setup() {
  // Backup copy only. From board firmware 3.2 the board's EEPROM is the truth
  // and adopt() replaces this on connect; older boards get this pushed to them.
  this->pref_ = global_preferences->make_preference<int32_t>(this->get_object_id_hash());
  if (!this->pref_.load(&this->steps_))
    this->steps_ = 0;

  // Length set from HA beats the YAML default. Different hash from the
  // position preference, so the two never collide.
  this->length_pref_ =
      global_preferences->make_preference<int32_t>(this->get_object_id_hash() ^ 0x4C454E47UL);
  int32_t saved_len;
  if (this->length_pref_.load(&saved_len) && saved_len >= 100)
    this->travel_steps_ = saved_len;

  this->publish_(this->steps_, cover::COVER_OPERATION_IDLE);
}

void AutoShadeCover::dump_config() {
  ESP_LOGCONFIG(TAG, "AUTOSHADE cover '%s':", this->get_name().c_str());
  ESP_LOGCONFIG(TAG, "  Motor: %u", this->motor_);
  ESP_LOGCONFIG(TAG, "  Travel: %d steps (%.1f in)", this->travel_steps_,
                this->travel_steps_ / this->steps_per_inch_);
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
    this->homing_approach_ = false;
    this->parent_->send_stop(this->motor_);
    return;
  }

  if (call.get_position().has_value()) {
    const float pos = *call.get_position();
    // position 1.0 = fully open = fully up = step 0
    const int32_t target =
        (int32_t) lroundf((1.0f - pos) * (float) this->travel_steps_);
    this->homing_ = false;
    this->last_cmd_ms_ = millis();
    this->parent_->send_move(this->motor_, target, this->max_speed_, this->accel_,
                             this->end_mask_());
  }
}

void AutoShadeCover::start_home() {
  if (this->parent_ == nullptr || this->home_overrun_ <= 0)
    return;
  this->homing_ = true;
  ESP_LOGI(TAG, "'%s': homing into the top stop", this->get_name().c_str());
  if (this->steps_ > this->home_overrun_) {
    // Stage 1: full speed up to just short of where we believe the top is.
    this->homing_approach_ = true;
    this->last_cmd_ms_ = millis();
    this->parent_->send_move(this->motor_, this->home_overrun_, this->max_speed_, this->accel_,
                             this->end_mask_());
  } else {
    this->home_final_();
  }
}

// Stage 2: quarter speed to an absolute target past zero, so the shade runs
// into the hard stop and skips there for at most 2 x home_overrun steps.
void AutoShadeCover::home_final_() {
  this->homing_approach_ = false;
  const uint16_t slow = this->max_speed_ / 4 > 20 ? this->max_speed_ / 4 : 20;
  this->last_cmd_ms_ = millis();
  this->parent_->send_move(this->motor_, -this->home_overrun_, slow, this->accel_,
                           this->end_mask_());
}

void AutoShadeCover::resync() {
  if (this->parent_ != nullptr)
    this->parent_->send_set_position(this->motor_, this->steps_);
}

void AutoShadeCover::jog(int32_t delta_steps) {
  if (this->parent_ == nullptr || delta_steps == 0)
    return;
  this->homing_ = false;
  this->homing_approach_ = false;
  // Jog is for finding the ends, so it goes at half speed for control.
  const uint16_t sps = this->max_speed_ / 2 > 20 ? this->max_speed_ / 2 : 20;
  this->last_cmd_ms_ = millis();
  this->parent_->send_move(this->motor_, this->steps_ + delta_steps, sps, this->accel_,
                           this->end_mask_());
}

bool AutoShadeCover::set_top_here() {
  if (this->parent_ == nullptr)
    return false;
  if (this->is_moving()) {
    ESP_LOGW(TAG, "'%s': still moving; wait for it to stop before Set Top", this->get_name().c_str());
    return false;
  }
  const int32_t new_len = this->travel_steps_ - this->steps_;
  if (new_len < 100) {
    ESP_LOGW(TAG, "'%s': that would leave %d steps of travel; ignored", this->get_name().c_str(),
             new_len);
    return false;
  }
  ESP_LOGI(TAG, "'%s': top set here (was step %d); length %d -> %d steps",
           this->get_name().c_str(), this->steps_, this->travel_steps_, new_len);
  this->parent_->send_set_position(this->motor_, 0);  // board saves it to EEPROM
  this->steps_ = 0;
  this->travel_steps_ = new_len;
  this->pref_.save(&this->steps_);
  this->save_length_();
  this->publish_(0, cover::COVER_OPERATION_IDLE);
  return true;
}

bool AutoShadeCover::set_bottom_here() {
  if (this->is_moving()) {
    ESP_LOGW(TAG, "'%s': still moving; wait for it to stop before Set Bottom",
             this->get_name().c_str());
    return false;
  }
  if (this->steps_ < 100) {
    ESP_LOGW(TAG, "'%s': only %d steps below the top; set the top first, or jog down further",
             this->get_name().c_str(), this->steps_);
    return false;
  }
  ESP_LOGI(TAG, "'%s': bottom set here; length %d -> %d steps", this->get_name().c_str(),
           this->travel_steps_, this->steps_);
  this->travel_steps_ = this->steps_;
  this->save_length_();
  this->publish_(this->steps_, cover::COVER_OPERATION_IDLE);
  return true;
}

bool AutoShadeCover::mark_position(float pos) {
  if (this->parent_ == nullptr)
    return false;
  if (this->is_moving()) {
    ESP_LOGW(TAG, "'%s': still moving; wait for it to stop before marking its position",
             this->get_name().c_str());
    return false;
  }
  const int32_t steps = (int32_t) lroundf((1.0f - clamp(pos, 0.0f, 1.0f)) * (float) this->travel_steps_);
  ESP_LOGI(TAG, "'%s': marked at %.0f%% (step %d, was %d)", this->get_name().c_str(), pos * 100.0f, steps,
           this->steps_);
  this->parent_->send_set_position(this->motor_, steps);  // board saves it to EEPROM
  this->steps_ = steps;
  this->homing_ = false;
  this->homing_approach_ = false;
  this->pref_.save(&this->steps_);
  this->publish_(steps, cover::COVER_OPERATION_IDLE);
  return true;
}

void AutoShadeCover::set_length_steps(int32_t steps) {
  if (steps < 100)
    steps = 100;
  ESP_LOGI(TAG, "'%s': length %d -> %d steps", this->get_name().c_str(), this->travel_steps_, steps);
  this->travel_steps_ = steps;
  this->save_length_();
  this->publish_(this->steps_, this->current_operation);
}

void AutoShadeCover::save_length_() {
  this->length_pref_.save(&this->travel_steps_);
  // Calibration is rare and annoying to redo: write it now rather than at the
  // next scheduled flash commit.
  global_preferences->sync();
  if (this->length_number_ != nullptr)
    this->length_number_->publish_state((float) this->travel_steps_ / this->steps_per_inch_);
}

void AutoShadeCover::adopt(int32_t steps) {
  this->steps_ = steps;
  this->have_steps_ = true;
  this->was_moving_ = false;
  this->homing_ = false;
  this->pref_.save(&this->steps_);
  this->publish_(steps, cover::COVER_OPERATION_IDLE);
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

  if (arrived && this->homing_ && this->homing_approach_) {
    this->steps_ = steps;
    this->publish_(steps, cover::COVER_OPERATION_OPENING);
    this->home_final_();
    return;
  }

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
