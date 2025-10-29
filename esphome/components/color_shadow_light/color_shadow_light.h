#pragma once

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <driver/ledc.h>

#include "esphome/components/light/light_output.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace color_shadow_light {

static constexpr float RED_TRIM = 0.95f;
static constexpr float GREEN_TRIM = 1.0f;
static constexpr float BLUE_TRIM = 0.40f;

class ColorShadowLight : public Component, public light::LightOutput {
 public:
  ColorShadowLight() { instance_ = this; }

  light::LightTraits get_traits() override {
    light::LightTraits traits;
    traits.set_supported_color_modes({light::ColorMode::RGB});
    return traits;
  }

  void setup() override {
    ledc_timer_config_t timer_config{};
    timer_config.speed_mode = LEDC_MODE;
    timer_config.timer_num = LEDC_TIMER_INDEX;
    timer_config.duty_resolution = static_cast<ledc_timer_bit_t>(PWM_RESOLUTION);
    timer_config.freq_hz = PWM_FREQUENCY;
    timer_config.clk_cfg = LEDC_USE_APB_CLK;
    ledc_timer_config(&timer_config);

    this->configure_channel_(RED_CHANNEL, BLUE_PIN);    // match legacy swapped wiring
    this->configure_channel_(GREEN_CHANNEL, GREEN_PIN);
    this->configure_channel_(BLUE_CHANNEL, RED_PIN);

    this->write_channel_(RED_CHANNEL, 0);
    this->write_channel_(GREEN_CHANNEL, 0);
    this->write_channel_(BLUE_CHANNEL, 0);

    this->safe_mode_pref_ = global_preferences->make_preference(sizeof(bool), SAFE_MODE_PREF_KEY, true);
    bool stored_safe_mode = true;
    if (!this->safe_mode_pref_.load(&stored_safe_mode)) {
      this->safe_mode_pref_.save(&stored_safe_mode);
      global_preferences->sync();
    }
    this->safe_mode_ = stored_safe_mode;
    this->current_power_limit_ = this->safe_mode_ ? LOCKED_POWER_LIMIT : UNLOCKED_POWER_LIMIT;

    analogSetAttenuation(ADC_2_5db);
    analogSetPinAttenuation(POT_RED_PIN, ADC_2_5db);
    analogSetPinAttenuation(POT_GREEN_PIN, ADC_2_5db);
    analogSetPinAttenuation(POT_BLUE_PIN, ADC_2_5db);

    this->sample_manual_inputs_(true);
  }

  void loop() override {
    const uint32_t now = millis();
    if (now - last_sample_time_ < UPDATE_INTERVAL_MS) {
      return;
    }
    last_sample_time_ = now;
    this->sample_manual_inputs_(false);
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Color Shadow Light");
    ESP_LOGCONFIG(TAG, "  Safe mode enabled: %s", this->safe_mode_ ? "true" : "false");
    ESP_LOGCONFIG(TAG, "  Power limit: %.2f", this->current_power_limit_);
  }

  void write_state(light::LightState *state) override {
    this->state_ = state;
    const auto remote = state->remote_values;
    const bool want_on = remote.is_on();

    if (!want_on) {
      this->is_on_ = false;
      this->manual_enabled_ = false;
      this->pending_manual_update_ = false;
      this->current_red_ = 0;
      this->current_green_ = 0;
      this->current_blue_ = 0;
      this->write_channel_(RED_CHANNEL, 0);
      this->write_channel_(GREEN_CHANNEL, 0);
      this->write_channel_(BLUE_CHANNEL, 0);
      return;
    }

    this->is_on_ = true;
    this->manual_enabled_ = true;

    this->apply_to_outputs_(remote.get_red(), remote.get_green(), remote.get_blue());

    if (this->pending_manual_update_) {
      this->pending_manual_update_ = false;
      this->publish_manual_update_(this->last_manual_red_, this->last_manual_green_, this->last_manual_blue_);
    }
  }

  void set_safe_mode(bool enabled) {
    if (this->safe_mode_ == enabled)
      return;
    this->safe_mode_ = enabled;
    this->current_power_limit_ = enabled ? LOCKED_POWER_LIMIT : UNLOCKED_POWER_LIMIT;
    this->safe_mode_pref_.save(&this->safe_mode_);
    global_preferences->sync();
    this->apply_current_state_();
  }

  bool safe_mode_enabled() const { return this->safe_mode_; }

  void reset_to_safe_mode() {
    this->set_safe_mode(true);
  }

  static ColorShadowLight *instance() { return instance_; }

 protected:
  light::LightState *state_{nullptr};

 private:
  static inline ColorShadowLight *instance_{nullptr};
  static constexpr const char *TAG = "color_shadow_light";

  static constexpr int POT_RED_PIN = 4;
  static constexpr int POT_GREEN_PIN = 3;
  static constexpr int POT_BLUE_PIN = 0;
  static constexpr int MOVING_AVERAGE_SIZE = 8;
  static constexpr uint32_t UPDATE_INTERVAL_MS = 20;
  static constexpr int ADC_MIN = 5;
  static constexpr int ADC_MAX = 950;
  static constexpr int PWM_RESOLUTION = 11;
  static constexpr int PWM_MAX = (1 << PWM_RESOLUTION) - 1;
  static constexpr int MANUAL_THRESHOLD = 25;
  static constexpr int OFF_THRESHOLD = 25;

  static constexpr float LOCKED_POWER_LIMIT = 0.3f;
  static constexpr float UNLOCKED_POWER_LIMIT = 0.6f;

  static constexpr int RED_PIN = 5;
  static constexpr int GREEN_PIN = 6;
  static constexpr int BLUE_PIN = 7;
  static constexpr ledc_channel_t RED_CHANNEL = LEDC_CHANNEL_0;
  static constexpr ledc_channel_t GREEN_CHANNEL = LEDC_CHANNEL_1;
  static constexpr ledc_channel_t BLUE_CHANNEL = LEDC_CHANNEL_2;
  static constexpr ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;
  static constexpr ledc_timer_t LEDC_TIMER_INDEX = LEDC_TIMER_0;
  static constexpr int PWM_FREQUENCY = 19000;
  static constexpr uint32_t SAFE_MODE_PREF_KEY = 0xC501A11F;

  float current_power_limit_{LOCKED_POWER_LIMIT};
  bool safe_mode_{true};
  bool manual_enabled_{false};
  bool pending_manual_update_{false};
  bool is_on_{false};

  int pot_red_values_[MOVING_AVERAGE_SIZE]{};
  int pot_green_values_[MOVING_AVERAGE_SIZE]{};
  int pot_blue_values_[MOVING_AVERAGE_SIZE]{};
  int pot_index_{0};

  int last_manual_red_{0};
  int last_manual_green_{0};
  int last_manual_blue_{0};

  int current_red_{0};
  int current_green_{0};
  int current_blue_{0};

  float update_threshold_{5.0f};
  float noise_threshold_{20.0f};
  float min_threshold_{5.0f};
  uint32_t last_change_time_{0};
  static constexpr uint32_t IDLE_TIME_THRESHOLD_MS = 7000;

  uint32_t last_sample_time_{0};
  ESPPreferenceObject safe_mode_pref_;

  void apply_current_state_() {
    if (this->state_ == nullptr) {
      return;
    }

    float red = 0.0f;
    float green = 0.0f;
    float blue = 0.0f;
    this->state_->current_values_as_rgb(&red, &green, &blue);
    this->apply_to_outputs_(red, green, blue);
  }

  void apply_to_outputs_(float red, float green, float blue) {
    int target_red = static_cast<int>(std::round(esphome::clamp(red, 0.0f, 1.0f) * PWM_MAX));
    int target_green = static_cast<int>(std::round(esphome::clamp(green, 0.0f, 1.0f) * PWM_MAX));
    int target_blue = static_cast<int>(std::round(esphome::clamp(blue, 0.0f, 1.0f) * PWM_MAX));

    target_red = static_cast<int>(target_red * RED_TRIM);
    target_green = static_cast<int>(target_green * GREEN_TRIM);
    target_blue = static_cast<int>(target_blue * BLUE_TRIM);

    target_red = static_cast<int>(target_red * this->current_power_limit_);
    target_green = static_cast<int>(target_green * this->current_power_limit_);
    target_blue = static_cast<int>(target_blue * this->current_power_limit_);

    target_red = esphome::clamp(target_red, 0, PWM_MAX);
    target_green = esphome::clamp(target_green, 0, PWM_MAX);
    target_blue = esphome::clamp(target_blue, 0, PWM_MAX);

    const bool update_red = this->should_update_(this->current_red_, target_red);
    const bool update_green = this->should_update_(this->current_green_, target_green);
    const bool update_blue = this->should_update_(this->current_blue_, target_blue);

    if (update_red) {
      this->current_red_ = target_red;
      this->write_channel_(RED_CHANNEL, this->current_red_);
    }
    if (update_green) {
      this->current_green_ = target_green;
      this->write_channel_(GREEN_CHANNEL, this->current_green_);
    }
    if (update_blue) {
      this->current_blue_ = target_blue;
      this->write_channel_(BLUE_CHANNEL, this->current_blue_);
    }
  }

  void sample_manual_inputs_(bool force_publish) {
    const int raw_red = this->read_averaged_adc_(POT_RED_PIN);
    const int raw_green = this->read_averaged_adc_(POT_GREEN_PIN);
    const int raw_blue = this->read_averaged_adc_(POT_BLUE_PIN);

    int mapped_red = map(constrain(raw_red, ADC_MIN, ADC_MAX), ADC_MIN, ADC_MAX, 0, PWM_MAX);
    int mapped_green = map(constrain(raw_green, ADC_MIN, ADC_MAX), ADC_MIN, ADC_MAX, 0, PWM_MAX);
    int mapped_blue = map(constrain(raw_blue, ADC_MIN, ADC_MAX), ADC_MIN, ADC_MAX, 0, PWM_MAX);

    pot_red_values_[pot_index_] = mapped_red;
    pot_green_values_[pot_index_] = mapped_green;
    pot_blue_values_[pot_index_] = mapped_blue;
    pot_index_ = (pot_index_ + 1) % MOVING_AVERAGE_SIZE;

    const int averaged_red = this->calculate_moving_average_(pot_red_values_);
    const int averaged_green = this->calculate_moving_average_(pot_green_values_);
    const int averaged_blue = this->calculate_moving_average_(pot_blue_values_);

    const bool red_changed = std::abs(averaged_red - last_manual_red_) > MANUAL_THRESHOLD;
    const bool green_changed = std::abs(averaged_green - last_manual_green_) > MANUAL_THRESHOLD;
    const bool blue_changed = std::abs(averaged_blue - last_manual_blue_) > MANUAL_THRESHOLD;

    if (force_publish || red_changed || green_changed || blue_changed) {
      last_manual_red_ = averaged_red;
      last_manual_green_ = averaged_green;
      last_manual_blue_ = averaged_blue;
      if (this->manual_enabled_) {
        this->publish_manual_update_(averaged_red, averaged_green, averaged_blue);
      } else {
        this->pending_manual_update_ = true;
      }
    }
  }

  void publish_manual_update_(int red, int green, int blue) {
    if (this->state_ == nullptr) {
      return;
    }

    this->pending_manual_update_ = false;
    int max_value = std::max({red, green, blue});
    auto call = this->state_->make_call();
    call.set_color_mode(light::ColorMode::RGB);
    call.set_transition_length(0);
    if (max_value <= OFF_THRESHOLD) {
      call.set_state(false);
    } else {
      call.set_state(true);
      call.set_red(static_cast<float>(red) / PWM_MAX);
      call.set_green(static_cast<float>(green) / PWM_MAX);
      call.set_blue(static_cast<float>(blue) / PWM_MAX);
    }
    call.perform();
  }

  int read_averaged_adc_(int pin, int samples = 4) {
    int32_t sum = 0;
    for (int i = 0; i < samples; i++) {
      sum += analogReadMilliVolts(pin);
    }
    return sum / samples;
  }

  int calculate_moving_average_(int *values) const {
    int32_t sum = 0;
    for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
      sum += values[i];
    }
    return static_cast<int>(sum / MOVING_AVERAGE_SIZE);
  }

  bool should_update_(int current, int next) {
    const int delta = std::abs(current - next);
    if (delta > noise_threshold_) {
      last_change_time_ = millis();
      update_threshold_ = min_threshold_;
      return true;
    }

    if (millis() - last_change_time_ > IDLE_TIME_THRESHOLD_MS) {
      update_threshold_ = noise_threshold_;
    }

    return delta > update_threshold_;
  }

  void configure_channel_(ledc_channel_t channel, int pin) {
    ledc_channel_config_t channel_config{};
    channel_config.gpio_num = pin;
    channel_config.speed_mode = LEDC_MODE;
    channel_config.channel = channel;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = LEDC_TIMER_INDEX;
    channel_config.duty = 0;
    channel_config.hpoint = 0;
    ledc_channel_config(&channel_config);
  }

  void write_channel_(ledc_channel_t channel, int duty) {
    ledc_set_duty(LEDC_MODE, channel, static_cast<uint32_t>(duty));
    ledc_update_duty(LEDC_MODE, channel);
  }
};

}  // namespace color_shadow_light
}  // namespace esphome
