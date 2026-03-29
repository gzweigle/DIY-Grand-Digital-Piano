// Copyright (C) 2025 Greg C. Zweigle
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
// https://github.com/gzweigle/open-hybrid-piano
// https://github.com/stem-piano
//
// src_hammer.cpp
//
// For ips pcb version 2.X
// For sca pcb version 0.0
//
// Main code for hammer board of stem piano.
//
// stem piano is an open source hybrid digital piano.
//

#include "stem_piano_ips2.h"

#include "hammer_settings.h"
#include "six_channel_analog_00.h"
#include "auto_mute.h"
#include "board2board.h"
#include "calibration_position.h"
#include "calibration_velocity.h"
#include "dsp_damper.h"
#include "dsp_hammer.h"
#include "dsp_pedal.h"
#include "hammer_status.h"
#include "midiout.h"
#include "network.h"
#include "nonvolatile.h"
#include "switches.h"
#include "testpoint_led.h"
#include "timing.h"
#include "tft_display.h"

HammerSettings Set;
SixChannelAnalog00 Adc;
AutoMute Mute;
Board2Board B2B;
CalibrationPosition CalP;
CalibrationVelocity CalV;
DspDamper DspD;
DspHammer DspH;
DspPedal DspP;
HammerStatus HStat;
MidiOut Midi;
Network Eth;
Nonvolatile Nonv;
Switches SwIPS1;
Switches SwIPS2;
Switches SwSCA1;
Switches SwSCA2;
TestpointLed Tpl;
Timing Tmg;
TftDisplay Tft;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, mi);
uint8_t Midi_Buffer[MIDI_BUFFER_SIZE];

void setup(void) {

  // Load the settings. Must be first in the setup() function.
  Set.SetAllSettingValues();

  partial_setup();
}

void partial_setup(void) {
  // Serial port setup.
  Serial.begin(9600);
  Serial.println(".");
  Serial.println(".");
  Serial.println("welcome to stem piano by gcz");
  Serial.println("an open source, hybrid piano");
  Serial.println(".");
  Serial.println(".");

  // Important information.
  Serial.println("Hardware required:");
  Serial.println("  IPS2.X hammer board.");
  Serial.println("  SCA0.0 analog board.");
  Serial.println("  HPS 0.8, 0.7, or 0.4 sensor board (any board meeting spec is also ok).");
  Serial.println("License: GNU GPLv3 - for documentation, code, and design see:");
  Serial.println("https://github.com/gzweigle/open-hybrid-piano");
  Serial.println("https://github.com/stem-piano");
  Serial.println(".");
  Serial.println(".");

  // Notification messages.
  if (Set.debug_level >= DEBUG_INFO) {
    Serial.println("Beginning hammer board initialization.");
  }
  Tft.Setup(Set.using_display, Set.debug_level);
  Tft.HelloWorld();

  // Initialize the nonvolatile memory.
  // Initialize early in case any setup() uses storage.
  Nonv.Setup(Set.debug_level);

  // Setup reading the switches.
  SwIPS1.Setup(Set.switch_debounce_micro,
  Set.switch11_ips_pin, Set.switch12_ips_pin, Set.debug_level);
  SwIPS2.Setup(Set.switch_debounce_micro,
  Set.switch21_ips_pin, Set.switch22_ips_pin, Set.debug_level);
  SwSCA1.Setup(Set.switch_debounce_micro,
  Set.switch11_sca_pin, Set.switch12_sca_pin, Set.debug_level);
  SwSCA2.Setup(Set.switch_debounce_micro,
  Set.switch21_sca_pin, Set.switch22_sca_pin, Set.debug_level);

  Mute.Setup(Set.maximum_midi_velocity);

  // Setup the analog front-end and the board-to-board communication.
  // Physically connecting the board-to-board link is optional and
  // only required if using a separate set of sensors for the dampers.
  // These two classes are common to hammer and pedal boards.
  Adc.Setup(Set.adc_spi_clock_frequency, Set.adc_is_differential,
  Set.using18bitadc, Set.sensor_v_max, Set.adc_reference,
  Set.adc_global_scale, Set.reorder_list, &Tpl);
  B2B.Setup(Set.canbus_enable);

  // Diagnostics and status
  HStat.Setup(&DspP, &Tpl, Set.debug_level);

  // Setup the dampers, hammers, and pedals on hammer board.
  DspD.Setup( Set.damper_threshold, Set.damper_velocity_scaling,
  Set.adc_sample_period_microseconds, Set.debug_level);
  DspH.Setup(Set.hammer_strike_algorithm, Set.adc_sample_period_microseconds,
  Set.strike_threshold, Set.release_threshold, Set.min_repetition_seconds,
  Set.min_strike_velocity, Set.hammer_travel_meters, Set.debug_level);
  DspP.Setup(Set.pedal_sample_interval_microseconds, Set.pedal_threshold,
  Set.sustain_pin, Set.sustain_connected_pin, Set.sostenuto_pin,
  Set.sostenuto_connected_pin, Set.una_corda_pin, Set.una_corda_connected_pin,
  Set.debug_level);

  // Adjust velocity based on the physical structure.
  CalV.Setup(Set.velocity_scale, Set.debug_level, &Nonv);

  // Setup sending damper, hammer, and pedal data over MIDI.
  Midi.Setup(Set.midi_channel, &mi, Set.maximum_midi_velocity, Set.debug_level);
  Serial1.addMemoryForWrite(Midi_Buffer, sizeof(Midi_Buffer));

  // Common on hammer and pedal board: Ethernet, test points, TFT display, etc.
  CalP.Setup(Set.calibration_threshold, Set.debug_level, &Nonv);
  Eth.Setup(Set.true_for_tcp_else_udp, Set.computer_ip, Set.teensy_ip, Set.network_port,
  SwIPS2.direct_read_switch_2(), Set.debug_level);
  Tpl.Setup();
  Tmg.Setup(Set.adc_sample_period_microseconds, Set.debug_level);

  if (Set.test_index >= 0) {
    Serial.println("WARNING - In high-speed test mode.");
    Serial.println("MIDI is disabled.");
  }

  // Ready to be a piano.
  if (Set.debug_level >= DEBUG_INFO) {
    Serial.println("Finished hammer board initialization.");
  }
  delay(1000);
  Tft.Clear();

  Serial.println("Press 1 to enter parameter edit mode.");
}

// After startup, wait before sending anything to MIDI
// in order to avoid any potential startup transients.
int startup_counter = 0;

// Switches.
bool switch_tft_display;
bool switch_external_damper_board;
bool switch_enable_ethernet;
bool switch_require_tcp_connection;
bool switch_enable_dynamic_velocity;
bool switch_freeze_cal_values;
bool switch_disable_and_reset_calibration;

// Data from ADC.
unsigned int raw_samples[NUM_CHANNELS];
unsigned int raw_samples_reordered[NUM_CHANNELS];
int hammer_adc_counts[NUM_CHANNELS];

// Damper, hammer, and pedal data.
bool damper_event[NUM_CHANNELS], hammer_event[NUM_CHANNELS];
float damper_position[NUM_CHANNELS], hammer_position[NUM_CHANNELS],
hammer_position_uncal[NUM_CHANNELS];
float damper_velocity[NUM_CHANNELS], hammer_velocity[NUM_CHANNELS];

int input_field = -1;
bool keep_editing = false;

void edit_parameters() {
  // effectively "infinite" timeout (~ 1 month) when waiting for input
  // in this function (parameter editing mode)
  Serial.setTimeout(2073600000L);

  Serial.println("Entered parameter editing mode.");
  keep_editing = true;
  while (keep_editing) {
    Serial.println("Which parameter would you like to edit?");
    Serial.println("0. NONE, done editing, resume normal operations");
    Serial.println("1. debug_level");
    Serial.println("2. sensor_v_max");
    Serial.println("3. adc_reference");
    Serial.println("4. adc_global_scale");
    Serial.println("5. velocity_scale");
    Serial.println("6. calibration_threshold");
    Serial.println("7. damper_threshold");
    Serial.println("8. damper_velocity_scaling");
    Serial.println("9. hammer_strike_algorithm");
    Serial.println("10. strike_threshold");
    Serial.println("11. release_threshold");
    Serial.println("12. min_repetition_seconds");
    Serial.println("13. min_strike_velocity");
    Serial.println("14. hammer_travel_meters");
    Serial.println("15. pedal_threshold");
    Serial.println("16. canbus_enable");

    if (Serial.available() > 0) {
      input_field = Serial.parseInt();
      switch (input_field) {
        case 0:
           // taken care of later
          break;
        case 1:
          // debug_level is INTEGER!
          Serial.print("Enter debug_level, currently ");
          Serial.println(Set.debug_level);
          Set.debug_level = Serial.parseInt();
          Serial.print("Set debug_level = ");
          Serial.println(Set.debug_level);
          break;
        case 2:
          Serial.print("Enter sensor_v_max, currently ");
          Serial.println(Set.sensor_v_max);
          Set.sensor_v_max = Serial.parseFloat();
          Serial.print("Set sensor_v_max = ");
          Serial.println(Set.sensor_v_max);
          break;
        case 3:
          Serial.print("Enter adc_reference, currently ");
          Serial.println(Set.adc_reference);
          Set.adc_reference = Serial.parseFloat();
          Serial.print("Set adc_reference = ");
          Serial.println(Set.adc_reference);
          break;
        case 4:
          Serial.print("Enter adc_global_scale, currently ");
          Serial.println(Set.adc_global_scale);
          Set.adc_global_scale = Serial.parseFloat();
          Serial.print("Set adc_global_scale = ");
          Serial.println(Set.adc_global_scale);
          break;
        case 5:
          Serial.print("Enter velocity_scale, currently ");
          Serial.println(Set.velocity_scale);
          Set.velocity_scale = Serial.parseFloat();
          Serial.print("Set velocity_scale = ");
          Serial.println(Set.velocity_scale);
          break;
        case 6:
          Serial.print("Enter calibration_threshold, currently ");
          Serial.println(Set.calibration_threshold);
          Set.calibration_threshold = Serial.parseFloat();
          Serial.print("Set calibration_threshold = ");
          Serial.println(Set.calibration_threshold);
          break;
        case 7:
          Serial.print("Enter damper_threshold, currently ");
          Serial.println(Set.damper_threshold);
          Set.damper_threshold = Serial.parseFloat();
          Serial.print("Set damper_threshold = ");
          Serial.println(Set.damper_threshold);
          break;
        case 8:
          Serial.print("Enter damper_velocity_scaling, currently ");
          Serial.println(Set.damper_velocity_scaling);
          Set.damper_velocity_scaling = Serial.parseFloat();
          Serial.print("Set damper_velocity_scaling = ");
          Serial.println(Set.damper_velocity_scaling);
          break;
        case 9:
          // hammer_strike_algorithm is INTEGER!
          Serial.print("Enter hammer_strike_algorithm, currently ");
          Serial.println(Set.hammer_strike_algorithm);
          Set.hammer_strike_algorithm = Serial.parseInt();
          Serial.print("Set hammer_strike_algorithm = ");
          Serial.println(Set.hammer_strike_algorithm);
          break;
        case 10:
          Serial.print("Enter strike_threshold, currently ");
          Serial.println(Set.strike_threshold);
          Set.strike_threshold = Serial.parseFloat();
          Serial.print("Set strike_threshold = ");
          Serial.println(Set.strike_threshold);
          break;
        case 11:
          Serial.print("Enter release_threshold, currently ");
          Serial.println(Set.release_threshold);
          Set.release_threshold = Serial.parseFloat();
          Serial.print("Set release_threshold = ");
          Serial.println(Set.release_threshold);
          break;
        case 12:
          Serial.print("Enter min_repetition_seconds, currently ");
          Serial.println(Set.min_repetition_seconds);
          Set.min_repetition_seconds = Serial.parseFloat();
          Serial.print("Set min_repetition_seconds = ");
          Serial.println(Set.min_repetition_seconds);
          break;
        case 13:
          Serial.print("Enter min_strike_velocity, currently ");
          Serial.println(Set.min_strike_velocity);
          Set.min_strike_velocity = Serial.parseFloat();
          Serial.print("Set min_strike_velocity = ");
          Serial.println(Set.min_strike_velocity);
          break;
        case 14:
          Serial.print("Enter hammer_travel_meters, currently ");
          Serial.println(Set.hammer_travel_meters);
          Set.hammer_travel_meters = Serial.parseFloat();
          Serial.print("Set hammer_travel_meters = ");
          Serial.println(Set.hammer_travel_meters);
          break;
        case 15:
          Serial.print("Enter pedal_threshold, currently ");
          Serial.println(Set.pedal_threshold);
          Set.pedal_threshold = Serial.parseFloat();
          Serial.print("Set pedal_threshold = ");
          Serial.println(Set.pedal_threshold);
          break;
        case 16:
          // canbus_enable is INTEGER!
          Serial.print("Enter canbus_enable, currently ");
          Serial.println(Set.canbus_enable);
          Set.canbus_enable = Serial.parseFloat();
          Serial.print("Set canbus_enable = ");
          Serial.println(Set.canbus_enable);
          break;

        default:
          Serial.print("Case ");
          Serial.print(input_field);
          Serial.println(" not supported");
          break;
      }
      if (input_field == 0) {
        keep_editing = false;
      }
    }
    delay(100);
  }

  // restore default timeout outside of parameter editing mode
  Serial.setTimeout(1000L);

  // re-run the set up with the changed values (may be unnecessary)
  Serial.println("Setting up the board with your changes");
  partial_setup();
}

void loop() {

  if (Serial.available() > 0) {
    input_field = Serial.parseInt(); // defaults to zero if times out
    if (input_field == 1) {
      edit_parameters();
    }
  }

  Tmg.WarnOnProcessingInterval();
  HStat.DisplayProcessingIntervalStart();

  // Measure every processing interval so the pickup and
  // dropout timers update. Later, when a switch is read,
  // what is actually read is the internal state of timers.
  SwIPS1.updatePuDoState("IPS11", "IPS12");
  SwIPS2.updatePuDoState("IPS21", "IPS22");
  SwSCA2.updatePuDoState("SCA21", "SCA22");
  SwSCA1.updatePuDoState("SCA11", "SCA12");

  // Read all switch inputs.

  // ips_sw1_position2 (ENABLE_TFT).
  switch_tft_display = SwIPS1.read_switch_2();

  // ips_sw1_position1 (EXTERNAL_DAMPER_BOARD).
  switch_external_damper_board = SwIPS1.read_switch_1();

  // ips_sw2_position2 (ENABLE_ETHERNET).
  switch_enable_ethernet = SwIPS2.read_switch_2();

  // ips_sw2_position1 (REQUIRE_TCP).
  switch_require_tcp_connection = SwIPS2.read_switch_1();

  // sca_sw2_position1 (DYNAMIC_VELOCITY_SCALING).
  switch_enable_dynamic_velocity = SwSCA2.read_switch_1();

  // sca_sw1_position2 (FREEZE_CAL_VALUES).
  switch_freeze_cal_values = SwSCA1.read_switch_2();

  // sca_sw1_positon1 (DELETE_CAL_VALUES).
  switch_disable_and_reset_calibration = SwSCA1.read_switch_1();

  // When the TFT is operational, turn off the alorithms that
  // could generate MIDI output and slow down the sampling.
  // Slow down sampling because the TFT takes a long time for
  // processing. But, do keep the sampling going so that the TFT
  // can display things like maximum and minimum hammer positions.
  if (switch_tft_display == true) {
    DspD.Enable(false);
    DspH.Enable(false);
    DspP.Enable(false);
    switch_freeze_cal_values = true; // Ignore switch value.
    Tmg.ResetInterval(Set.adc_sample_period_microseconds_during_tft);
  }
  else {
    DspD.Enable(true);
    DspH.Enable(true);
    DspP.Enable(true);
    Tmg.ResetInterval(Set.adc_sample_period_microseconds);
  }

  // This if statement determines the sample rate.
  // Everything below in this file runs at the sample rate.
  if (Tmg.AllowProcessing() == true) {

    Tpl.SetTp8(true); // Front left test point asserts during processing.

    // Get hammer and pedal data from ADC.
    Adc.GetNewAdcValues(raw_samples, Set.test_index);

    // Reorder the back row.
    Adc.ReorderAdcValues(raw_samples_reordered, raw_samples);

    // Normalize the ADC values.
    Adc.NormalizeAdcValues(hammer_adc_counts, hammer_position_uncal, raw_samples_reordered);

    // Undo the position errors due to physical tolerances.
    bool all_notes_using_cal = CalP.Calibration(switch_freeze_cal_values,
    switch_disable_and_reset_calibration, hammer_position, hammer_position_uncal);

    // Zero out the signal from any unconnected pins to avoid noise
    // or anything else causing an unwanted piano note to play.
    for (int k = 0; k < NUM_CHANNELS; k++) {
      if (Set.connected_channel[k] == false) {
        hammer_adc_counts[k] = 0;
        hammer_position[k] = 0.0;
      }
    }

    if (Set.test_index < 0) {

      // Select source for damper signals.
      if (switch_external_damper_board == true) {
        // There exists a damper board so use remote data from the damper board.
        B2B.GetDamperData(damper_position);
      }
      else {
        // No external damper board.
        // Use the hammer position as an estimate of the damper position.
        for (int k = 0; k < NUM_CHANNELS; k++)
          damper_position[k] = hammer_position[k];
      }

      // Process hammer, damper, and pedal data.
      // For hammer and damper get an event boolean flag and velocity.
      // For pedal get the state of the pedal.
      DspH.GetHammerEventData(hammer_event, hammer_velocity, hammer_position);
      DspD.GetDamperEventData(damper_event, damper_velocity, damper_position);
      DspD.CheckHammerDamperSync(damper_event, damper_velocity, damper_position,
      hammer_event);
      DspP.UpdatePedalState(hammer_position);

      // Adjust velocity because each physical setup is different.
      CalV.DamperVelocityScale(damper_velocity, damper_event);
      CalV.HammerVelocityScale(hammer_velocity, hammer_event,
      switch_enable_dynamic_velocity, switch_freeze_cal_values,
      switch_disable_and_reset_calibration, all_notes_using_cal); 

      // Sending data over MIDI.
      if (startup_counter < Set.startup_counter_value) {
        startup_counter++;
      }
      else {
        Midi.SendNoteOn(&Mute, hammer_event, hammer_velocity);
        Midi.SendNoteOff(&Mute, damper_event, damper_velocity,
          switch_external_damper_board);
        Midi.SendPedal(&DspP);
      }
    }

    Eth.SendPianoPacket(hammer_position, damper_position,
      switch_enable_ethernet, switch_require_tcp_connection,
      Set.test_index);

    if (Set.test_index < 0) {
      // Run the TFT display.
      Tft.Display(switch_tft_display, hammer_position, damper_position);
    }

    // Debug and display information.
    HStat.FrontLed(hammer_position, Set.damper_threshold,
    Set.strike_threshold, Set.test_index);

    if (Set.test_index < 0) {
      HStat.LowerRightLed(all_notes_using_cal, Nonv.NonvolatileWasWritten());
      HStat.SCALed();
      HStat.EthernetLed();
      HStat.SerialMonitor(hammer_adc_counts, hammer_position, hammer_event,
      Set.canbus_enable, switch_external_damper_board);
    }

    Tpl.SetTp8(false);
  }

  HStat.DisplayProcessingIntervalEnd();
}
