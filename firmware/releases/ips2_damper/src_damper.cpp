// Copyright (C) 2023 Greg C. Zweigle
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
// src_damper.cpp
//
// For ips pcb version 2.X
// For sca pcb version 0.0
//
// Main code for damper board of Stem Piano.
//
// TODO - For more than 88 keys, could use local hammer for extra
//        dampers, and remote for the normal 88.

#include "stem_piano_ips2.h"

#include "damper_settings.h"
#include "six_channel_analog_00.h"
#include "board2board.h"
#include "network.h"
#include "switches.h"
#include "testpoint_led.h"
#include "timing.h"
#include "tft_display.h"

DamperSettings Set;

SixChannelAnalog00 Adc;
Board2Board B2B;
Network Eth;
Switches SwIPS1;
Switches SwIPS2;
Switches SwSCA1;
Switches SwSCA2;
TestpointLed Tpl;
Timing Tmg;
TftDisplay Tft;

void setup(void) {

  // This must be placed first and in the setup() function.
  Set.SetAllSettingValues();

  // These must be called second and in the setup() function.
  SwIPS1.Setup(Set.switch_debounce_micro,
  Set.switch11_ips_pin, Set.switch12_ips_pin, Set.debug_level);
  SwIPS2.Setup(Set.switch_debounce_micro,
  Set.switch21_ips_pin, Set.switch22_ips_pin, Set.debug_level);
  SwSCA1.Setup(Set.switch_debounce_micro,
  Set.switch11_sca_pin, Set.switch12_sca_pin, Set.debug_level);
  SwSCA2.Setup(Set.switch_debounce_micro,
  Set.switch21_sca_pin, Set.switch22_sca_pin, Set.debug_level);

  Serial.begin(Set.serial_baud_rate);

  if (Set.debug_level >= 1) {
    Serial.println("Beginning damper board initialization.");
  }

  // Setup everything.
  Adc.Setup(Set.adc_spi_clock_frequency, Set.adc_is_differential, &Tpl);
  B2B.Setup(Set.canbus_enable);
  Eth.Setup(Set.computer_ip, Set.teensy_ip, Set.upd_port, Set.debug_level);
  Tpl.Setup();
  Tmg.Setup(Set.adc_sample_period_microseconds);
  Tft.Setup(Set.using_display, Set.debug_level);
  Tft.HelloWorld();

  if (Set.debug_level >= 1) {
    Serial.println("Finished damper board initialization.");
  }

  Serial.println("GNU GPLv3 - for documentation, code, and design see:");
  Serial.println("https://github.com/gzweigle/DIY-Grand-Digital-Piano");

  Serial.println(".");
  Serial.println("Welcome to Stem Piano, IPS2.X/SCA0.0 Damper Board, by GCZ.");
  Serial.println(".");

}

// Temporary debug and diagnostic stuff.
unsigned long last_micros_ = micros();
bool serial_display_toggle = false;

// Place declaration of all variables for DIP switches here.
bool switch_tft;

// Data from ADC.
unsigned int raw_samples[NUM_CHANNELS], reordered_raw_samples[NUM_CHANNELS];
int damper_position_adc_counts[NUM_CHANNELS];
float damper_position[NUM_CHANNELS];

void loop() {

  // Measure every processing interval so the pickup and
  // dropout timers update. In other places, when a switch
  // is read, what is actually read is the internal state
  // of timers.
  SwIPS1.updatePuDoState("IPS11", "IPS12");
  SwIPS2.updatePuDoState("IPS21", "IPS22");
  SwSCA1.updatePuDoState("SCA11", "SCA12");
  SwSCA2.updatePuDoState("SCA21", "SCA22");

  // Read all switch values in one place so can keep track of them.
  switch_tft = SwIPS2.read_switch_1();  // TFT mode.

  // When the TFT is operational, turn off the alorithms that
  // could generate MIDI output and slow down the sampling.
  // Slow down sampling because the TFT takes a long time for
  // processing. But, do keep the sampling going so that the TFT
  // can display things like maximum and minimum hammer positions.
  if (switch_tft == true) {
    Tmg.ResetInterval(Set.adc_sample_period_microseconds_during_tft);
  }
  else {
    Tmg.ResetInterval(Set.adc_sample_period_microseconds);
  }

  // This if statement determines the sample rate.
  if (Tmg.AllowProcessing() == true) {
    Tpl.SetTp8(true);
    
    Adc.GetNewAdcValues(raw_samples);

    // Reorder the inputs
    for (int ind = 0; ind < NUM_CHANNELS; ind++) {
        reordered_raw_samples[Set.note_order[ind]] = raw_samples[ind];
    }

    Adc.NormalizeAdcValues(damper_position_adc_counts, damper_position,
    reordered_raw_samples);
    for (int k = 0; k < NUM_CHANNELS; k++) {
      if (Set.connected_channel[k] == false) {
        damper_position_adc_counts[k] = 0;
        damper_position[k] = 0.0;
      }
    }
    B2B.SendDamperData(damper_position);

    Eth.SendPianoPacket(damper_position[32],
                        damper_position[33],
                        damper_position[34],
                        damper_position[35],
                        damper_position[36],
                        damper_position[37],
                        damper_position[38],
                        damper_position[39]);

    // Run the TFT display.
    Tft.Display(switch_tft, damper_position, damper_position);

    /////////////////
    // Temporary debug and diagnostic stuff.
    // Flash all the LED to make sure they are working.
    // Except tp8 and tp9 as they are used elsewhere.
    if (micros() - last_micros_ > Set.serial_display_interval) {
      last_micros_ = micros();
      if (serial_display_toggle == true) {
        serial_display_toggle = false;
        Tpl.SetTp10(false);
        Tpl.SetTp11(true);
        Tpl.SetLowerRightLED(false);
        Tpl.SetScaLedL(false);
        Tpl.SetScaLedR(false);
        Tpl.SetEthernetLED(false);
      }
      else {
        serial_display_toggle = true;
        if (Set.debug_level >= 1) {
          Serial.print("Damper Board ");
          if (Set.board_bringup == true) {
            Serial.print(" d24="); Serial.print(damper_position[24]);
            Serial.print(" d25="); Serial.print(damper_position[25]);
            Serial.print(" d26="); Serial.print(damper_position[26]);
            Serial.print(" d27="); Serial.print(damper_position[27]);
          }
          else
          {
            Serial.print(" d32="); Serial.print(damper_position[32]);
            Serial.print(" d33="); Serial.print(damper_position[33]);
            Serial.print(" d34="); Serial.print(damper_position[34]);
            Serial.print(" d35="); Serial.print(damper_position[35]);
            Serial.print(" d36="); Serial.print(damper_position[36]);
            Serial.print(" d37="); Serial.print(damper_position[37]);
            Serial.print(" d38="); Serial.print(damper_position[38]);
            Serial.print(" d39="); Serial.print(damper_position[39]);
          }
          Serial.println("");
        }
        Tpl.SetTp10(true);
        Tpl.SetTp11(false);
        Tpl.SetLowerRightLED(true);
        Tpl.SetScaLedL(true);
        Tpl.SetScaLedR(true);
        Tpl.SetEthernetLED(true);
      }
    }
    /////////////////

    Tpl.SetTp8(false);
  }
}