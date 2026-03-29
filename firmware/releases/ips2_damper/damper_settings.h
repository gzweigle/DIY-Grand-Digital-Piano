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
// damper_settings.h
//
// For ips pcb version 2.X
// For sca pcb version 0.0
//
// Common location for all settings related to the system.

#ifndef DAMPER_SETTINGS_H_
#define DAMPER_SETTINGS_H_

#include "stem_piano_ips2.h"

#define IP_STRING_LENGTH 128

// uncomment the next line if the dampers sensors are under the keys
// rather than above
// #define REVERSE_DAMPERS

class DamperSettings
{
  
  public:

    DamperSettings();
    void SetAllSettingValues();
    int debug_level;
    int startup_counter_value;
    int adc_spi_clock_frequency;
    int test_index;
    int adc_sample_period_microseconds;
    int adc_sample_period_microseconds_during_tft;
    bool adc_is_differential;
    bool using18bitadc;
    float sensor_v_max;
    float adc_reference;
    float adc_global_scale;
    int switch_debounce_micro;
    int switch11_ips_pin;
    int switch12_ips_pin;
    int switch21_ips_pin;
    int switch22_ips_pin;
    int switch11_sca_pin;
    int switch12_sca_pin;
    int switch21_sca_pin;
    int switch22_sca_pin;
    float calibration_threshold;
    bool true_for_tcp_else_udp;
    char teensy_ip[IP_STRING_LENGTH];
    char computer_ip[IP_STRING_LENGTH];
    int network_port;
    bool canbus_enable;
    bool using_display;
    bool connected_channel[NUM_CHANNELS];
    int reorder_list[NUM_CHANNELS];
};

#endif
