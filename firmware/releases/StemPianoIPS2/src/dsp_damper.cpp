// Copyright (C) 2024 Greg C. Zweigle
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// Location of documentation, code, and design:
// https://github.com/gzweigle/DIY-Grand-Digital-Piano
//
// dsp_damper.cpp
//
// This class is not hardware dependent.
//
// Damper digital signal processing

#include "dsp_damper.h"

DspDamper::DspDamper() {}

void DspDamper::Setup(float damper_threshold_low, float damper_threshold_high,
float velocity_scaling, int adc_sample_period_microseconds, int debug_level) {

  debug_level_ = debug_level;

  // Hysteresis to force one event around a threshold crossing.
  // Initialize to a large value to avoid startup transients.
  for (int key = 0; key < NUM_CHANNELS; key++) {
    event_block_counter_[key] = 2*(NUM_DELAY_ELEMENTS);
  }

  // Delay buffer, for lowpass filtering the velocity.
  buffer_index_ = 0;
  for (int key = 0; key < NUM_CHANNELS; key++) {
    for (int ind = 0; ind < NUM_DELAY_ELEMENTS; ind++) {
      damper_buffer_[key][ind] = 0.0;
    }
  }

  damper_threshold_low_ = damper_threshold_low;
  damper_threshold_high_ = damper_threshold_high;
  velocity_scaling_ = velocity_scaling;
  samples_per_second_ = 1000000.0 /static_cast<float>(adc_sample_period_microseconds);
  enable_ = true;
}

// When position[key] crosses a threshold, set event[key] true and store the 
// associated damper velocity in velocity[key].
// The velocity[key] value will be > 0 on damper rising and < 0 on damper falling.
void DspDamper::GetDamperEventData(bool *event, float *velocity, const float *position,
bool switch_high_damper_threshold) {

  float damper_threshold;
  if (switch_high_damper_threshold == true) {
    damper_threshold = damper_threshold_high_;
  }
  else {
    damper_threshold = damper_threshold_low_;
  }

  if (enable_ == true) {

    float position_now, position_then;  // Use to keep code easier to read.

    for (int key = 0; key < NUM_CHANNELS; key++) {

      // Present and oldest position values for each key.
      position_now = position[key];
      position_then = damper_buffer_[key][(buffer_index_+1)%(NUM_DELAY_ELEMENTS)];
      damper_buffer_[key][buffer_index_] = position[key];

      if (event_block_counter_[key] == 0) {
        // If crossed a threshold going up, velocity will be positive.
        // If crossed a threshold going down, velocity will be negative.
        // In all cases, velocity is in range [-1, ..., 1].
        if (position_now >= damper_threshold &&
        position_then < damper_threshold)
        {
          event[key] = false;
          velocity[key] = (position_now - position_then) *
          samples_per_second_ / static_cast<float>(NUM_DELAY_ELEMENTS);
          velocity[key] *= velocity_scaling_;
          event_block_counter_[key] = 2*(NUM_DELAY_ELEMENTS);
          if (debug_level_ >= DEBUG_ALL) {
            Serial.println("GetDamperEventData() - damper up");
            Serial.print("  key=");
            Serial.print(key);
            Serial.print(" pos_now=");
            Serial.print(position_now);
            Serial.print(" pos_then=");
            Serial.print(position_then);
            Serial.print(" velocity=");
            Serial.print(velocity[key]);
            Serial.print(" threshold=");
            Serial.print(damper_threshold);
            Serial.println("");
          }
        }
        else if (position_now <= damper_threshold &&
        position_then > damper_threshold)
        {
          event[key] = true;  // Damp the sound.
          velocity[key] = (position_now - position_then) *
          samples_per_second_ / static_cast<float>(NUM_DELAY_ELEMENTS);
          velocity[key] *= velocity_scaling_;
          event_block_counter_[key] = 2*(NUM_DELAY_ELEMENTS);
          if (debug_level_ >= DEBUG_ALL) {
            Serial.println("GetDamperEventData() - damper down");
            Serial.print("  key=");
            Serial.print(key);
            Serial.print(" pos_now=");
            Serial.print(position_now);
            Serial.print(" pos_then=");
            Serial.print(position_then);
            Serial.print(" velocity=");
            Serial.print(velocity[key]);
            Serial.print(" threshold=");
            Serial.print(damper_threshold);
            Serial.println("");
          }
        }
        else {
          event[key] = false;
          velocity[key] = 0.0;
        }
      }
      else if (event_block_counter_[key] > 0) {
        // Using this counter to block back-to-back events,
        // in case there is any jitter in signal around
        // a threshold crossing.
        event_block_counter_[key]--;
        event[key] = false;
      }
    }
    buffer_index_ = (buffer_index_+1)%(NUM_DELAY_ELEMENTS);

  }
  else {
    for (int key = 0; key < NUM_CHANNELS; key++) {
      event[key] = false;
    }
  }
}

void DspDamper::Enable(bool enable) {
  enable_ = enable;
}