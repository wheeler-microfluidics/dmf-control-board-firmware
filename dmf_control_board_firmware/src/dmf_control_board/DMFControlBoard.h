/*
Copyright 2011 Ryan Fobel

This file is part of dmf_control_board.

dmf_control_board is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

dmf_control_board is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with dmf_control_board.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _DMF_CONTROL_BOARD_H_
#define _DMF_CONTROL_BOARD_H_

#if !( defined(AVR) || defined(__SAM3X8E__) )
  #include <iostream>
  #include <fstream>
  #include <string>
  #include <vector>
  #include <stdint.h>
  #include <stdexcept>
#else
  #include "Arduino.h"
  #include "Config.h"
  #include "FeedbackController.h"
#endif
#include "RemoteObject.h"


class DMFControlBoard : public RemoteObject {
public:
  static const uint8_t SINE = 0;
  static const uint8_t SQUARE = 1;

  // impedance measurement options (bit definitions)
  // IMPOPT: - - - - - - RMS INTLV
  static const uint8_t INTLV = 0;
  static const uint8_t RMS = 1;

#if ___HARDWARE_MAJOR_VERSION___ == 1
  static const uint8_t WAVEFORM_SELECT_ = 9;
#endif

  /**\brief Address of config settings in persistent storage _(i.e., EEPROM)_.
   */
  static const uint16_t PERSISTENT_CONFIG_SETTINGS = 100;

  struct version_t {
    uint16_t major;
    uint16_t minor;
    uint16_t micro;
  };

#if defined(AVR) || defined(__SAM3X8E__)
  struct watchdog_t {
    /* # `watchdog_t` #
     *
     * This structure is used to maintain the state of a watchdog timer, which
     * can be used for any purpose.  For now, the watchdog timer is used to
     * maintain the state of the ATX power supply control signal.
     *
     * If the watchdog timer is enabled, _i.e., `enabled=true`, when
     * the timer period finishes:
     *
     *  - If the `state` is not `true`, the power-supply must be
     *   turned off.
     *  - The `state` must be set to `false`.
     *
     * When the watchdog timer is enabled, it is the responsibility of the
     * client to reset the watchdog-state before each timer period ends to
     * prevent the power supply from being turned off. */
    bool enabled;
    bool state;

    watchdog_t() : enabled(false), state(false) {}
  };

  struct ConfigSettings {
    uint8_t n_series_resistors(uint8_t channel) {
      switch(channel) {
        case 0:
          return sizeof(A0_series_resistance)/sizeof(float);
          break;
        case 1:
          return sizeof(A1_series_resistance)/sizeof(float);
          break;
        // we should never get here
        default:
          return 0;
          break;
      }
    }

    float series_resistance(uint8_t channel, uint8_t index) {
      switch(channel) {
        case 0:
          return A0_series_resistance[index];
          break;
        case 1:
          return A1_series_resistance[index];
          break;
        // we should never get here
        default:
          return 0;
          break;
      }
    }

    float series_capacitance(uint8_t channel, uint8_t index) {
      switch(channel) {
        case 0:
          return A0_series_capacitance[index];
          break;
        case 1:
          return A1_series_capacitance[index];
          break;
        // we should never get here
        default:
          return 0;
          break;
      }
    }

    /**\brief This is the software version that the persistent configuration
     * data was written with.*/
    version_t version;

    #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ < 3
      /**\brief This byte sets the value of the analog reference (between 0 and
      ~5V).*/
      uint8_t aref;
    #endif

    /**\brief i2c address of first high-voltage switching board.*/
    uint8_t switching_board_i2c_address;

    #if ___HARDWARE_MAJOR_VERSION___ == 1
      /**\brief This byte sets the maximum output voltage for the waveform
      generator. It should be trimmed so that the output waveform is 4Vp-p when
      POT_WAVEOUT_GAIN_2 is set to 255.*/
      uint8_t waveout_gain_1;

      /**\brief This byte sets the value of the virtual ground reference
      (between 0 and 5V).*/
      uint8_t vgnd;

      /**\brief Series resistor values for channel 0.*/
      float A0_series_resistance[2];
      /**\brief Series capacitance values for channel 0.*/
      float A0_series_capacitance[2];
      /**\brief Series resistor values for channel 1.*/
      float A1_series_resistance[4];
      /**\brief Series capacitance values for channel 1.*/
      float A1_series_capacitance[4];
    #else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
      /**\brief i2c address of first signal generator board.*/
      uint8_t signal_generator_board_i2c_address;
      /**\brief Series resistor values for channel 0.*/
      float A0_series_resistance[3];
      /**\brief Series capacitance values for channel 0.*/
      float A0_series_capacitance[3];
      /**\brief Series resistor values for channel 1.*/
      float A1_series_resistance[5];
      /**\brief Series capacitance values for channel 1.*/
      float A1_series_capacitance[5];
    #endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1

    /**\brief Amplifier gain (if >0). If <=0, the gain is automatically
    adjusted based on the measured voltage from the amplifier.*/
    float amplifier_gain;

    /**\brief Voltage tolerance for amplifier gain adjustment.*/
    float voltage_tolerance;

    /**\brief Use anti-aliasing filter.*/
    bool use_antialiasing_filter;

    /**\brief Valid waveform frequency range.*/
    float waveform_frequency_range[2];

    /**\brief Maximum valid waveform voltage.*/
    float max_waveform_voltage;
  };
#endif  // #if defined(AVR) || defined(__SAM3X8E__)

  // Accessors and mutators
  static const uint8_t CMD_GET_NUMBER_OF_CHANNELS =         0xA0;
  static const uint8_t CMD_GET_STATE_OF_ALL_CHANNELS =      0xA1;
  static const uint8_t CMD_SET_STATE_OF_ALL_CHANNELS =      0xA2;
  static const uint8_t CMD_GET_STATE_OF_CHANNEL =           0xA3;
  static const uint8_t CMD_SET_STATE_OF_CHANNEL =           0xA4;
  static const uint8_t CMD_GET_WAVEFORM =                   0xA5;
  static const uint8_t CMD_SET_WAVEFORM =                   0xA6;
  static const uint8_t CMD_GET_WAVEFORM_VOLTAGE =           0xA7;
  static const uint8_t CMD_SET_WAVEFORM_VOLTAGE =           0xA8;
  static const uint8_t CMD_GET_WAVEFORM_FREQUENCY =         0xA9;
  static const uint8_t CMD_SET_WAVEFORM_FREQUENCY =         0xAA;
  static const uint8_t CMD_GET_SERIES_RESISTOR_INDEX =      0xAD;
  static const uint8_t CMD_SET_SERIES_RESISTOR_INDEX =      0xAE;
  static const uint8_t CMD_GET_SERIES_RESISTANCE =          0xAF;
  static const uint8_t CMD_SET_SERIES_RESISTANCE =          0xB0;
  static const uint8_t CMD_GET_SERIES_CAPACITANCE =         0xB1;
  static const uint8_t CMD_SET_SERIES_CAPACITANCE =         0xB2;
  static const uint8_t CMD_GET_AMPLIFIER_GAIN =             0xB3;
  static const uint8_t CMD_SET_AMPLIFIER_GAIN =             0xB4;
  static const uint8_t CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN = 0xB5;
  static const uint8_t CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN = 0xB6;
  static const uint8_t CMD_GET_POWER_SUPPLY_PIN =           0xB7;
  static const uint8_t CMD_GET_WATCHDOG_STATE =             0xB8;
  static const uint8_t CMD_SET_WATCHDOG_STATE =             0xB9;
  static const uint8_t CMD_GET_WATCHDOG_ENABLED =           0xBA;
  static const uint8_t CMD_SET_WATCHDOG_ENABLED =           0xBB;
  static const uint8_t CMD_GET_ATX_POWER_STATE =            0xBC;
  static const uint8_t CMD_SET_ATX_POWER_STATE =            0xBD;

  // Other commands
  static const uint8_t CMD_SYSTEM_RESET =                   0xF1; //TODO
  static const uint8_t CMD_DEBUG_MESSAGE =                  0xF2; //TODO
  static const uint8_t CMD_DEBUG_ON =                       0xF3; //TODO
  static const uint8_t CMD_MEASURE_IMPEDANCE =              0xF4;
  static const uint8_t CMD_LOAD_CONFIG =                    0xF5;

  //////////////////////////////////////////////////////////////////////////////
  //
  // Return codes:
  //
  // Return codes are uint8_t.  Some generic return codes are defined by the
  // RemoteObject class.  You may also create custom return codes as long as
  // they are > 0x1F.
  //
  //////////////////////////////////////////////////////////////////////////////

  static const uint8_t RETURN_MAX_CURRENT_EXCEEDED =        0x20;

  DMFControlBoard();
  ~DMFControlBoard();

// In our case, the PC is the only one sending commands
#if !( defined(AVR) || defined(__SAM3X8E__) )
  virtual std::string command_label(uint8_t command) const {
    try {
      return RemoteObject::command_label(command);
    } catch (...) {
      if (command == CMD_GET_NUMBER_OF_CHANNELS) {
        return std::string("CMD_GET_NUMBER_OF_CHANNELS");
      } else if (command == CMD_GET_STATE_OF_ALL_CHANNELS) {
        return std::string("CMD_GET_STATE_OF_ALL_CHANNELS");
      } else if (command == CMD_SET_STATE_OF_ALL_CHANNELS) {
        return std::string("CMD_SET_STATE_OF_ALL_CHANNELS");
      } else if (command == CMD_GET_STATE_OF_CHANNEL) {
        return std::string("CMD_GET_STATE_OF_CHANNEL");
      } else if (command == CMD_SET_STATE_OF_CHANNEL) {
        return std::string("CMD_SET_STATE_OF_CHANNEL");
      } else if (command == CMD_GET_WAVEFORM) {
        return std::string("CMD_GET_WAVEFORM");
      } else if (command == CMD_SET_WAVEFORM) {
        return std::string("CMD_SET_WAVEFORM");
      } else if (command == CMD_GET_WAVEFORM_VOLTAGE) {
        return std::string("CMD_GET_WAVEFORM_VOLTAGE");
      } else if (command == CMD_SET_WAVEFORM_VOLTAGE) {
        return std::string("CMD_SET_WAVEFORM_VOLTAGE");
      } else if (command == CMD_GET_WAVEFORM_FREQUENCY) {
        return std::string("CMD_GET_WAVEFORM_FREQUENCY");
      } else if (command == CMD_SET_WAVEFORM_FREQUENCY) {
        return std::string("CMD_SET_WAVEFORM_FREQUENCY");
      } else if (command == CMD_GET_SERIES_RESISTOR_INDEX) {
        return std::string("CMD_GET_SERIES_RESISTOR_INDEX");
      } else if (command == CMD_SET_SERIES_RESISTOR_INDEX) {
        return std::string("CMD_SET_SERIES_RESISTOR_INDEX");
      } else if (command == CMD_GET_SERIES_RESISTANCE) {
        return std::string("CMD_GET_SERIES_RESISTANCE");
      } else if (command == CMD_SET_SERIES_RESISTANCE) {
        return std::string("CMD_SET_SERIES_RESISTANCE");
      } else if (command == CMD_GET_SERIES_CAPACITANCE) {
        return std::string("CMD_GET_SERIES_CAPACITANCE");
      } else if (command == CMD_SET_SERIES_CAPACITANCE) {
        return std::string("CMD_SET_SERIES_CAPACITANCE");
      } else if (command == CMD_GET_AMPLIFIER_GAIN) {
        return std::string("CMD_GET_AMPLIFIER_GAIN");
      } else if (command == CMD_SET_AMPLIFIER_GAIN) {
        return std::string("CMD_SET_AMPLIFIER_GAIN");
      } else if (command == CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN) {
        return std::string("CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN");
      } else if (command == CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN) {
        return std::string("CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN");
      } else if (command == CMD_GET_POWER_SUPPLY_PIN) {
        return std::string("CMD_GET_POWER_SUPPLY_PIN");
      } else if (command == CMD_GET_WATCHDOG_STATE) {
        return std::string("CMD_GET_WATCHDOG_STATE");
      } else if (command == CMD_SET_WATCHDOG_STATE) {
        return std::string("CMD_SET_WATCHDOG_STATE");
      } else if (command == CMD_GET_WATCHDOG_ENABLED) {
        return std::string("CMD_GET_WATCHDOG_ENABLED");
      } else if (command == CMD_SET_WATCHDOG_ENABLED) {
        return std::string("CMD_SET_WATCHDOG_ENABLED");
      } else if (command == CMD_GET_ATX_POWER_STATE) {
        return std::string("CMD_GET_ATX_POWER_STATE");
      } else if (command == CMD_SET_ATX_POWER_STATE) {
        return std::string("CMD_SET_ATX_POWER_STATE");
      } else {
        throw std::runtime_error("Invalid command.");
      }
    }
  }

  uint16_t number_of_channels();
  std::vector<uint8_t> state_of_all_channels();
  uint8_t state_of_channel(const uint16_t channel);
  uint8_t series_resistor_index(const uint8_t channel);
  float series_resistance(const uint8_t channel);
  float series_capacitance(const uint8_t channel);
  std::string waveform();
  float waveform_frequency();
  float waveform_voltage();
  float amplifier_gain();
  bool auto_adjust_amplifier_gain();
  uint8_t power_supply_pin();
  bool watchdog_state();
  bool watchdog_enabled();
  bool atx_power_state();

  // Remote mutators (return code is from reply packet)
  uint8_t set_state_of_channel(const uint16_t channel, const uint8_t state);
  uint8_t set_state_of_all_channels(const std::vector<uint8_t> state);
  uint8_t set_waveform_voltage(const float v_rms);
  uint8_t set_waveform_frequency(const float freq_hz);
  uint8_t set_waveform(bool waveform);
  uint8_t set_series_resistor_index(const uint8_t channel,
                                    const uint8_t index);
  uint8_t set_series_resistance(const uint8_t channel,
                                float resistance);
  uint8_t set_series_capacitance(const uint8_t channel,
                                 float capacitance);
  uint8_t set_amplifier_gain(float gain);
  uint8_t set_auto_adjust_amplifier_gain(bool on);
  uint8_t set_watchdog_state(bool state);
  uint8_t set_watchdog_enabled(bool state);
  uint8_t set_atx_power_state(bool state);

  // other functions
  void measure_impedance_non_blocking(float sampling_window_ms,
                                      uint16_t n_sampling_windows,
                                      float delay_between_windows_ms,
                                      bool interleave_samples,
                                      bool rms,
                                      const std::vector<uint8_t> state);
  std::vector<float> get_impedance_data();
  std::vector<float> measure_impedance(float sampling_window_ms,
                                       uint16_t n_sampling_windows,
                                       float delay_between_windows_ms,
                                       bool interleave_samples,
                                       bool rms,
                                       const std::vector<uint8_t> state);
  uint8_t reset_config_to_defaults() { return load_config(true); }
  uint8_t load_config(bool use_defaults);

  std::string host_name() { return NAME_; }
  std::string host_manufacturer() { return MANUFACTURER_; }
  std::string host_software_version() { return SOFTWARE_VERSION_; }
  std::string host_url() { return URL_; }
#else  // #ifndef AVR
  void begin();
  void on_connect();
  uint8_t set_waveform_voltage(const float output_vrms,
                               const bool wait_for_reply=true);
  uint8_t set_waveform_frequency(const float frequency);
  void set_amplifier_gain(float gain);

  // local accessors
  ConfigSettings config_settings() { return config_settings_; }
  const char* protocol_name() { return PROTOCOL_NAME_; }
  const char* protocol_version() { return PROTOCOL_VERSION_; }
  const char* name() { return NAME_; } //device name
  const char* manufacturer() { return MANUFACTURER_; }
  const char* software_version() { return SOFTWARE_VERSION_; }
  const char* hardware_version();
  const char* url() { return URL_; }
  float waveform_voltage() { return waveform_voltage_; }
  float waveform_frequency() { return waveform_frequency_; }
  bool auto_adjust_amplifier_gain() { return auto_adjust_amplifier_gain_; }
  float amplifier_gain() { return amplifier_gain_; }
  /* Note that the ATX power-supply output-enable is _active-low_. */
  void atx_power_on() const {
    digitalWrite(POWER_SUPPLY_ON_PIN_, LOW);
    delay(500);
  }
  void atx_power_off() const { digitalWrite(POWER_SUPPLY_ON_PIN_, HIGH); }
  bool atx_power_state() const { return !digitalRead(POWER_SUPPLY_ON_PIN_); }
  bool watchdog_enabled() const { return watchdog_.enabled; }
  void set_watchdog_enabled(bool state) { watchdog_.enabled = state; }
  bool watchdog_state() const { return watchdog_.state; }
  void set_watchdog_state(bool state) { watchdog_.state = state; }
  void watchdog_reset() { set_watchdog_state(true); }
  void clear_all_channels();

  /* Callback function when watchdog timer period is finished. */
  void watchdog_timeout() {
    /* If the watchdog timer is enabled, _i.e., `enabled=true`, when the timer
     * period finishes:
     *
     *  - If the `state` is not `true`, the power-supply must be turned off.
     *  - The `state` must be set to `false`.
     *
     * When the watchdog timer is enabled, it is the responsibility of the
     * client to reset the watchdog-state before each timer period ends to
     * prevent the power supply from being turned off. */
    if (watchdog_enabled()) {
      if (!watchdog_state()) {
        /* Watchdog timer has timed out and the state has not been reset since
         * the previous timeout, so execute error handler. */
        watchdog_error();
      } else {
        /* Watchdog timer has timed out, but the state has been reset.  Prepare
         * for next timeout event. */
        set_watchdog_state(false);
      }
    }
  }

  virtual void watchdog_error() {
    atx_power_off();
    connected_ = false;
  }

  /* Expose to allow timer callback to check state. */
  watchdog_t watchdog_;
#endif  // #ifndef AVR

private:
  // private static members
  static const char SOFTWARE_VERSION_[];
  static const char NAME_[];
  static const char HARDWARE_VERSION_[];
  static const char MANUFACTURER_[];
  static const char URL_[];
#if defined(AVR) || defined(__SAM3X8E__)
  static const char PROTOCOL_NAME_[];
  static const char PROTOCOL_VERSION_[];

  static const uint8_t AD5204_SLAVE_SELECT_PIN_ = 53; // digital pot

  #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ < 3
    static const uint8_t POT_INDEX_AREF_ = 0;
  #endif

  #if ___HARDWARE_MAJOR_VERSION___ == 1
    static const uint8_t POT_INDEX_VGND_ = 1;
    static const uint8_t POT_INDEX_WAVEOUT_GAIN_1_ = 2;
    static const uint8_t POT_INDEX_WAVEOUT_GAIN_2_ = 3;
  #endif

  #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ > 1
    static const uint8_t POWER_SUPPLY_ON_PIN_ = 8;
  #elif ___HARDWARE_MAJOR_VERSION___ == 2
    static const uint8_t POWER_SUPPLY_ON_PIN_ = 2;
  #endif

  // I2C bus
  // =======
  // A4 SDA
  // A5 SCL

  // SPI bus
  // =======
  // D10 SS
  // D11 MOSI
  // D13 SCK

  // PCA9505 (gpio) chip/register addresses
  static const uint8_t PCA9505_CONFIG_IO_REGISTER_ = 0x18;
  static const uint8_t PCA9505_OUTPUT_PORT_REGISTER_ = 0x08;

  // LTC6904 (programmable oscillator) chip address
  static const uint8_t LTC6904_ = 0x17;

#else  // #ifdef AVR
  static const char CSV_INDENT_[];
#endif  // #ifdef AVR

  // private functions
  virtual uint8_t process_command(uint8_t cmd);
#if defined(AVR) || defined(__SAM3X8E__)
  uint8_t update_channel(const uint16_t channel, const uint8_t state);
  void update_all_channels();
  void send_spi(uint8_t pin, uint8_t address, uint8_t data);
  uint8_t set_pot(uint8_t index, uint8_t value);
  void load_config(bool use_defaults=false);
  void save_config();
  version_t config_version();
#endif

  //private members
#if defined(AVR) || defined(__SAM3X8E__)
  bool connected_;
  uint16_t number_of_channels_;
  FeedbackController feedback_controller_;
  float waveform_voltage_;
  float waveform_frequency_;
  float amplifier_gain_;
  bool auto_adjust_amplifier_gain_;
  ConfigSettings config_settings_;
#endif  // #ifdef AVR
};
#endif // _DMF_CONTROL_BOARD_H_

