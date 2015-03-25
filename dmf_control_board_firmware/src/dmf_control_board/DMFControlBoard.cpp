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

#include <stdint.h>
#include "DMFControlBoard.h"
#if !( defined(AVR) || defined(__SAM3X8E__) )
  #include <boost/format.hpp>
  using namespace std;
  using boost::format;
#else
  #include "Arduino.h"
  #include <Wire.h>
  #include <OneWire.h>
  #include <SPI.h>
  #ifdef AVR
    #include <EEPROM.h>
  #elif defined(__SAM3X8E__)
    #include <DueFlashStorage.h>
    extern DueFlashStorage EEPROM;
  #endif
  #include <math.h>
  #include <AdvancedADC.h>
#endif

#if defined(AVR) || defined(__SAM3X8E__)
  const char DMFControlBoard::PROTOCOL_NAME_[] = "DMF Control Protocol";
  const char DMFControlBoard::PROTOCOL_VERSION_[] = "0.1";

  void printe(float number) {
    int8_t exp = floor(log10(number));
    Serial.print(number / pow(10, exp));
    Serial.print("E");
    Serial.print(exp, DEC);
  }

  void printlne(float number) {
    printe(number);
    Serial.println();
  }
#else
  const char DMFControlBoard::CSV_INDENT_[] = ",,,,,,,,";
#endif

const char DMFControlBoard::NAME_[] = "Arduino DMF Controller";
const char DMFControlBoard::MANUFACTURER_[] = "Wheeler Microfluidics Lab";
const char DMFControlBoard::SOFTWARE_VERSION_[] = ___SOFTWARE_VERSION___;
const char DMFControlBoard::URL_[] =
    "http://microfluidics.utoronto.ca/dmf_control_board";

DMFControlBoard::DMFControlBoard()
  : RemoteObject(true
#if !( defined(AVR) || defined(__SAM3X8E__) )
                   ,"DMFControlBoard"  //used for logging
#endif
  ) {
}

DMFControlBoard::~DMFControlBoard() {
}

uint8_t DMFControlBoard::process_command(uint8_t cmd) {
#if !( defined(AVR) || defined(__SAM3X8E__) )
  const char* function_name = "process_command()";
  log_message(str(format("command=0x%0X (%d)") % cmd % cmd).c_str(),
              function_name);
#else
  watchdog_reset();
  if (!connected_) {
    on_connect();
  }
#endif
  switch(cmd) {
#if defined(AVR) || defined(__SAM3X8E__) // Commands that only the Arduino handles
    case CMD_GET_NUMBER_OF_CHANNELS:
      if (payload_length() == 0) {
        serialize(&number_of_channels_, sizeof(number_of_channels_));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_STATE_OF_ALL_CHANNELS:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        for (uint8_t chip = 0; chip < number_of_channels_ / 40; chip++) {
          for (uint8_t port = 0; port < 5; port++) {
            Wire.beginTransmission(config_settings_.switching_board_i2c_address
                                   + chip);
            Wire.write(PCA9505_OUTPUT_PORT_REGISTER_ + port);
            Wire.endTransmission();
            Wire.requestFrom(
              config_settings_.switching_board_i2c_address + chip, 1);
            if (Wire.available()) {
              uint8_t data = Wire.read();
              uint8_t state;
              for (uint8_t bit = 0; bit < 8; bit++) {
                state = (data >> bit & 0x01) == 0;
                serialize(&state, sizeof(state));
              }
            } else {
              return_code_ = RETURN_GENERAL_ERROR;
              break; break;
            }
          }
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_STATE_OF_ALL_CHANNELS:
      if (payload_length() == number_of_channels_ * sizeof(uint8_t)) {
        update_all_channels();
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_STATE_OF_CHANNEL:
      if (payload_length() == sizeof(uint16_t)) {
        uint16_t channel = read_uint16();
        if (channel >= number_of_channels_ || channel < 0) {
          return_code_ = RETURN_BAD_INDEX;
        } else {
          uint8_t chip = channel / 40;
          uint8_t port = (channel % 40) / 8;
          uint8_t bit = (channel % 40) % 8;
          Wire.beginTransmission(
            config_settings_.switching_board_i2c_address + chip);
          Wire.write(PCA9505_OUTPUT_PORT_REGISTER_ + port);
          Wire.endTransmission();
          Wire.requestFrom(
            config_settings_.switching_board_i2c_address + chip, 1);
          if (Wire.available()) {
            uint8_t data = Wire.read();
            data = (data >> bit & 0x01) == 0;
            serialize(&data, sizeof(data));
            return_code_ = RETURN_OK;
          } else {
            return_code_ = RETURN_GENERAL_ERROR;
          }
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_STATE_OF_CHANNEL:
      if (payload_length() == sizeof(uint16_t) + sizeof(uint8_t)) {
        uint16_t channel = read_uint16();
        uint8_t state = read_uint8();
        if (channel < number_of_channels_) {
          return_code_ = update_channel(channel, state);
        } else {
          return_code_ = RETURN_BAD_INDEX;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#if ___HARDWARE_MAJOR_VERSION___ == 1
    case CMD_GET_WAVEFORM:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        uint8_t waveform = digitalRead(WAVEFORM_SELECT_);
        serialize(&waveform, sizeof(waveform));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WAVEFORM:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t waveform = read_uint8();
        if (waveform == SINE || waveform == SQUARE) {
          digitalWrite(WAVEFORM_SELECT_, waveform);
          return_code_ = RETURN_OK;
        } else {
          return_code_ = RETURN_BAD_VALUE;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#endif  //#if ___HARDWARE_MAJOR_VERSION___ == 1
    case CMD_GET_WAVEFORM_VOLTAGE:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        serialize(&waveform_voltage_, sizeof(waveform_voltage_));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WAVEFORM_VOLTAGE:
      if (payload_length() == sizeof(float)) {
        return_code_ = set_waveform_voltage(read_float());
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_WAVEFORM_FREQUENCY:
      if (payload_length() == 0) {
#if ___HARDWARE_MAJOR_VERSION___ == 1
        return_code_ = RETURN_OK;
        serialize(&waveform_frequency_, sizeof(waveform_frequency_));
#else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
        i2c_write(config_settings_.signal_generator_board_i2c_address, cmd);
        delay(I2C_DELAY);
        Wire.requestFrom(config_settings_.signal_generator_board_i2c_address,
                         (uint8_t)1);
        if (Wire.available()) {
          uint8_t n_bytes_to_read = Wire.read();
          if (n_bytes_to_read == sizeof(float) + 1) {
              uint8_t data[5];
              i2c_read(config_settings_.signal_generator_board_i2c_address,
                       (uint8_t * )&data[0], 5);
              return_code_ = data[4];
              if (return_code_ == RETURN_OK) {
                memcpy(&waveform_frequency_, &data[0], sizeof(float));
                serialize(&waveform_frequency_, sizeof(float));
              }
            }
          }
#endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WAVEFORM_FREQUENCY:
      if (payload_length() == sizeof(float)) {
        return_code_ = set_waveform_frequency(read_float());
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_SERIES_RESISTOR_INDEX:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t channel = read_uint8();
        if (channel <= 1) {
          uint8_t index = feedback_controller_.series_resistor_index(channel);
          return_code_ = RETURN_OK;
          serialize(&index, sizeof(uint8_t));
        } else {
          return_code_ = RETURN_BAD_INDEX;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SERIES_RESISTOR_INDEX:
      if (payload_length() == 2*sizeof(uint8_t)) {
        uint8_t channel = read_uint8();
        return_code_ = feedback_controller_.set_series_resistor_index(channel,
            read_uint8());
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_SERIES_RESISTANCE:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t channel = read_uint8();
        if (channel < 2) {
          return_code_ = RETURN_OK;
          float resistance = config_settings_.series_resistance(channel,
              feedback_controller_.series_resistor_index(channel));
          serialize(&resistance, sizeof(resistance));
        } else {
            return_code_ = RETURN_BAD_INDEX;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SERIES_RESISTANCE:
      if (payload_length() == sizeof(uint8_t) + sizeof(float)) {
        uint8_t channel = read_uint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            config_settings_.A0_series_resistance[ \
              feedback_controller_.series_resistor_index(channel)] = \
                read_float();
            save_config();
            break;
          case 1:
            config_settings_.A1_series_resistance[ \
              feedback_controller_.series_resistor_index(channel)] = \
                read_float();
            save_config();
            break;
          default:
            return_code_ = RETURN_BAD_INDEX;
            break;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_SERIES_CAPACITANCE:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t channel = read_uint8();
        if (channel < 2) {
          return_code_ = RETURN_OK;
          float capacitance = config_settings_.series_capacitance(channel,
            feedback_controller_.series_resistor_index(channel));
          serialize(&capacitance, sizeof(capacitance));
        } else {
            return_code_ = RETURN_BAD_INDEX;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SERIES_CAPACITANCE:
      if (payload_length() == sizeof(uint8_t) + sizeof(float)) {
        uint8_t channel = read_uint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            config_settings_.A0_series_capacitance[ \
              feedback_controller_.series_resistor_index(channel)] = \
                read_float();
            save_config();
            break;
          case 1:
            config_settings_.A1_series_capacitance[ \
              feedback_controller_.series_resistor_index(channel)] = \
                read_float();
            save_config();
            break;
          default:
            return_code_ = RETURN_BAD_INDEX;
            break;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_AMPLIFIER_GAIN:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        serialize(&amplifier_gain_, sizeof(amplifier_gain_));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_AMPLIFIER_GAIN:
      if (payload_length() == sizeof(float)) {
        float value = read_float();
        if (value > 0) {
          if (auto_adjust_amplifier_gain_) {
            amplifier_gain_ = value;
          } else {
            config_settings_.amplifier_gain = value;
            auto_adjust_amplifier_gain_ = false;
            save_config();
          }
          return_code_ = RETURN_OK;
        } else {
          return_code_ = RETURN_BAD_VALUE;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        uint8_t value = config_settings_.amplifier_gain <= 0;
        serialize(&value, sizeof(value));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t value = read_uint8();
        if (value > 0) {
          config_settings_.amplifier_gain = 0;
        } else {
          config_settings_.amplifier_gain = amplifier_gain_;
        }
        save_config();
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_MEASURE_IMPEDANCE:
      if (payload_length() < (sizeof(uint16_t) + 2*sizeof(float))) {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      } else {
        float sampling_window_ms = read_float();
        uint16_t n_sampling_windows = read_uint16();
        float delay_between_windows_ms = read_float();
        uint8_t options = read_uint8();

        // decode impedance option bits
        // IMPOPT: - - - - - - RMS INTLV
        bool interleave_samples = (options & (1 << INTLV)) > 0;
        bool rms =  (options & (1 << RMS)) > 0;

        // command packet can optionally include state of the channels
        if (payload_length() == (sizeof(uint8_t) + sizeof(uint16_t) + \
            2 * sizeof(float)) || \
            (payload_length() == (sizeof(uint8_t) + sizeof(uint16_t) + \
                2 * sizeof(float) + number_of_channels_ * sizeof(uint8_t)))) {
          // make sure that the number of sampling windows doesn't exceed the
          // limits of the output buffer and that the sampling window length
          // will not overflow the sum^2 variables
          if ((n_sampling_windows <= (MAX_PAYLOAD_LENGTH - 4 * sizeof(float)) /
                (FeedbackController::NUMBER_OF_ADC_CHANNELS * \
                 (sizeof(int8_t) + sizeof(int16_t)))) && \
                 (sampling_window_ms / 1000 * \
                   feedback_controller_.MAX_SAMPLING_RATE < 4096)) {
            return_code_ = RETURN_OK;

            // update the channels (if they were included in the packet)
            if (payload_length() == (sizeof(uint8_t) + sizeof(uint16_t) + \
                2 * sizeof(float) + number_of_channels_ * sizeof(uint8_t))) {
              update_all_channels();
            }

            long start_time = micros();
            return_code_ = feedback_controller_.measure_impedance(
                                                   sampling_window_ms,
                                                   n_sampling_windows,
                                                   delay_between_windows_ms,
                                                   waveform_frequency_,
                                                   interleave_samples,
                                                   rms);
            if (return_code_ == RETURN_OK) {
              // return the time between sampling windows (dt) in ms
              float dt_ms = (float)(micros() - start_time) / \
                  (1000.0 * n_sampling_windows);
              serialize(&dt_ms, sizeof(dt_ms));

              // return the vgnd values
              for (uint8_t index = 0;
                   index < FeedbackController::NUMBER_OF_ADC_CHANNELS;
                   index++) {
                serialize(&
                  feedback_controller_.channels()[index].vgnd_exp_filtered,
                  sizeof(float));
              }
              // return the amplifier gain
              float gain = amplifier_gain();
              serialize(&gain, sizeof(float));
            }
          } else {
            return_code_ = RETURN_GENERAL_ERROR;
          }
        } else {
          return_code_ = RETURN_BAD_PACKET_SIZE;
        }
      }
      break;
    case CMD_LOAD_CONFIG:
      if (payload_length() == 1) {
        return_code_ = RETURN_OK;
        bool use_defaults = read_uint8() > 0;
        load_config(use_defaults);
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_POWER_SUPPLY_PIN:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        uint8_t power_supply_pin = POWER_SUPPLY_ON_PIN_;
        serialize(&power_supply_pin, sizeof(power_supply_pin));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_WATCHDOG_ENABLED:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        uint8_t value = watchdog_enabled();
        serialize(&value, sizeof(value));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WATCHDOG_ENABLED:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t value = read_uint8();
        if (value > 0) {
          set_watchdog_enabled(true);
        } else {
          set_watchdog_enabled(false);
        }
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_WATCHDOG_STATE:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        uint8_t value = watchdog_state();
        serialize(&value, sizeof(value));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WATCHDOG_STATE:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t value = read_uint8();
        if (value > 0) {
          set_watchdog_state(true);
        } else {
          set_watchdog_state(false);
        }
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_ATX_POWER_STATE:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        uint8_t value = atx_power_state();
        serialize(&value, sizeof(value));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_ATX_POWER_STATE:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t value = read_uint8();
        if (value > 0) {
          atx_power_on();
        } else {
          atx_power_off();
        }
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#endif  // #ifdef AVR
  }
  RemoteObject::process_command(cmd);
#if !( defined(AVR) || defined(__SAM3X8E__) )
  if (return_code_ == RETURN_UNKNOWN_COMMAND) {
    log_error("Unrecognized command", function_name);
  }
#endif
  return return_code_;
}

#if defined(AVR) || defined(__SAM3X8E__)
///////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the Arduino.
//
///////////////////////////////////////////////////////////////////////////////

void DMFControlBoard::begin() {
  RemoteObject::begin();

  // Set the POWER_SUPPLY_ON_PIN_ HIGH. Otherwise, the power supply turns on
  // when we set the pin as an output (i.e., LOW is the default state).
  digitalWrite(POWER_SUPPLY_ON_PIN_, HIGH);
  pinMode(POWER_SUPPLY_ON_PIN_, OUTPUT);

#if ___HARDWARE_MAJOR_VERSION___ == 1
  pinMode(AD5204_SLAVE_SELECT_PIN_, OUTPUT);
  pinMode(WAVEFORM_SELECT_, OUTPUT);
  // set waveform (SINE=0, SQUARE=1)
  digitalWrite(WAVEFORM_SELECT_, SINE);
#endif

  // default amplifier gain
  amplifier_gain_ = 300;

  load_config();

  // set all digital pots

  // Versions > 1.2 use the built in 5V AREF
  #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ < 3
    set_pot(POT_INDEX_AREF_, config_settings_.aref);
  #endif
  #if ___HARDWARE_MAJOR_VERSION___ == 1
    set_pot(POT_INDEX_VGND_, config_settings_.vgnd);
    set_pot(POT_INDEX_WAVEOUT_GAIN_1_, config_settings_.waveout_gain_1);
    set_pot(POT_INDEX_WAVEOUT_GAIN_2_, 0);
  #endif

  Serial.print(name());
  Serial.print(" v");
  Serial.println(hardware_version());
  Serial.print("Firmware version: ");
  Serial.println(software_version());

  Serial.print("Configuration version=");
  Serial.print(config_settings_.version.major, DEC);
  Serial.print(".");
  Serial.print(config_settings_.version.minor, DEC);
  Serial.print(".");
  Serial.println(config_settings_.version.micro, DEC);

  // Versions > 1.2 use the built in 5V AREF
  #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ < 3
    Serial.print("aref=");
    Serial.println(config_settings_.aref, DEC);
  #endif

  #if ___HARDWARE_MAJOR_VERSION___ == 1
    Serial.print("waveout_gain_1=");
    Serial.println(config_settings_.waveout_gain_1, DEC);
    Serial.print("vgnd=");
    Serial.println(config_settings_.vgnd, DEC);
  #else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
    Serial.print("signal_generator_board_i2c_address=");
    Serial.println(config_settings_.signal_generator_board_i2c_address, DEC);
  #endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else

  for (uint8_t i=0; i < 2; i++) {
    for (uint8_t j=0; j < config_settings_.n_series_resistors(i); j++) {
      Serial.print("A" + String(i) + "_series_resistance[" + String(j) \
          + "]=");
      Serial.println(config_settings_.series_resistance(i, j));
      Serial.print("A" + String(i) + "_series_capacitance[" + String(j) \
          + "]=");
      Serial.print(1e12*config_settings_.series_capacitance(i, j));
      Serial.println(" pF");
    }
  }
  Serial.print("switching_board_i2c_address=");
  Serial.println(config_settings_.switching_board_i2c_address, DEC);
  Serial.print("amplifier_gain=");
  Serial.println(config_settings_.amplifier_gain);
  Serial.print("voltage_tolerance=");
  Serial.println(config_settings_.voltage_tolerance);
  Serial.print("use_antialiasing_filter=");
  Serial.println(config_settings_.use_antialiasing_filter);
  Serial.print("auto_adjust_amplifier_gain=");
  Serial.println(auto_adjust_amplifier_gain_);
  Serial.print("min_waveform_frequency=");
  Serial.println(config_settings_.waveform_frequency_range[0]);
  Serial.print("max_waveform_frequency=");
  Serial.println(config_settings_.waveform_frequency_range[1]);
  Serial.print("max_waveform_voltage=");
  Serial.println(config_settings_.max_waveform_voltage);
}

void DMFControlBoard::on_connect() {
  connected_ = true;

  // turn on the power supply
  atx_power_on();

  // Check how many switching boards are connected.  Each additional board's
  // address must equal the previous boards address +1 to be valid.
  number_of_channels_ = 0;

  uint8_t data[2];
  for (uint8_t chip = 0; chip < 8; chip++) {
    // set IO ports as inputs
    data[0] = PCA9505_CONFIG_IO_REGISTER_;
    data[1] = 0xFF;
    i2c_write(config_settings_.switching_board_i2c_address + chip,
              data, 2);

    // read back the register value
    i2c_read(config_settings_.switching_board_i2c_address + chip,
             data, 1);

    // if it matches what we previously set, this might be a PCA9505 chip
    if (data[0] == 0xFF) {
      // try setting all ports in output mode and initialize to ground
      uint8_t port=0;
      for (; port<5; port++) {
        data[0] = PCA9505_CONFIG_IO_REGISTER_ + port;
        data[1] = 0x00;
        i2c_write(config_settings_.switching_board_i2c_address + chip,
                  data, 2);
        i2c_read(config_settings_.switching_board_i2c_address + chip,
                  data, 1);

        // check that we successfully set the IO config register to 0x00
        if (data[0] != 0x00) {
          break;
        }
        data[0] = PCA9505_OUTPUT_PORT_REGISTER_ + port;
        data[1] = 0xFF;
        i2c_write(config_settings_.switching_board_i2c_address + chip,
                  data, 2);
      }

      // if port=5, it means that we successfully initialized all IO config
      // registers to 0x00, and this is probably a PCA9505 chip
      if (port==5) {
        Serial.print("HV board ");
        Serial.print((int)chip);
        Serial.println(" connected.");
        if (number_of_channels_ == 40 * chip) {
          number_of_channels_ = 40 * (chip + 1);
        }
      }
    }
  }
  Serial.print(number_of_channels_);
  Serial.println(" channels available.");

  // set the default frequency to 10kHz
  set_waveform_frequency(10e3);
}

const char* DMFControlBoard::hardware_version() {
  return ___HARDWARE_VERSION___;
}

void DMFControlBoard::send_spi(uint8_t pin, uint8_t address, uint8_t data) {
  digitalWrite(pin, LOW);
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(pin, HIGH);
}

uint8_t DMFControlBoard::set_pot(uint8_t index, uint8_t value) {
  if (index>=0 && index<4) {
    send_spi(AD5204_SLAVE_SELECT_PIN_, index, 255-value);
    return RETURN_OK;
  }
  return RETURN_BAD_INDEX;
}

// update the state of all channels
void DMFControlBoard::update_all_channels() {
  // Each PCA9505 chip has 5 8-bit output registers for a total of 40 outputs
  // per chip. We can have up to 8 of these chips on an I2C bus, which means
  // we can control up to 320 channels.
  //   Each register represent 8 channels (i.e. the first register on the
  // first PCA9505 chip stores the state of channels 0-7, the second register
  // represents channels 8-15, etc.).
  uint8_t data[2];
  for (uint8_t chip=0; chip<number_of_channels_/40; chip++) {
    for (uint8_t port = 0; port < 5; port++) {
      data[0] = PCA9505_OUTPUT_PORT_REGISTER_ + port;
      data[1] = 0;
      for (uint8_t i = 0; i < 8; i++) {
        data[1] += (read_uint8() == 0) << i;
      }
      i2c_write(config_settings_.switching_board_i2c_address + chip,
                data, 2);
    }
  }
}

// clear the state of all channels
void DMFControlBoard::clear_all_channels() {
  // See update_all_channels
  uint8_t data[2];
  for (uint8_t chip=0; chip<number_of_channels_/40; chip++) {
    for (uint8_t port = 0; port < 5; port++) {
      data[0] = PCA9505_OUTPUT_PORT_REGISTER_ + port;
      data[1] = 0xFF;
      i2c_write(config_settings_.switching_board_i2c_address + chip,
                data, 2);
    }
  }
}

// Update the state of single channel.
// Note: Do not use this function in a loop to update all channels. If you
//       want to update all channels, use the update_all_channels function
//       instead because it will be 8x more efficient.
uint8_t DMFControlBoard::update_channel(const uint16_t channel,
                                        const uint8_t state) {
  uint8_t chip = channel / 40;
  uint8_t port = (channel % 40) / 8;
  uint8_t bit = (channel % 40) % 8;
  Wire.beginTransmission(
    config_settings_.switching_board_i2c_address + chip);
  Wire.write(PCA9505_OUTPUT_PORT_REGISTER_ + port);
  Wire.endTransmission();
  Wire.requestFrom(
    config_settings_.switching_board_i2c_address + chip, 1);
  if (Wire.available()) {
    uint8_t data[2];
    data[0] = PCA9505_OUTPUT_PORT_REGISTER_ + port;
    data[1] = Wire.read();
    bitWrite(data[1], bit, state == 0);
    i2c_write(config_settings_.switching_board_i2c_address + chip,
              data, 2);
    return RETURN_OK;
  } else {
    return RETURN_GENERAL_ERROR;
  }
}

DMFControlBoard::version_t DMFControlBoard::config_version() {
  version_t config_version;
  uint8_t* p = (uint8_t*)&config_version;
  for (uint16_t i = 0; i < sizeof(version_t); i++) {
    p[i] = this->persistent_read(PERSISTENT_CONFIG_SETTINGS + i);
  }
  return config_version;
}

void DMFControlBoard::load_config(bool use_defaults) {
  uint8_t* p = (uint8_t*)&config_settings_;
  for (uint16_t i = 0; i < sizeof(ConfigSettings); i++) {
    p[i] = this->persistent_read(PERSISTENT_CONFIG_SETTINGS + i);
  }

  float default_voltage_tolerance = 5.0;
  float default_min_waveform_frequency = 100;
  float default_max_waveform_frequency = 20e3;
  float default_max_waveform_voltage = 200;

  // Upgrade config settings if necessary
  if (config_settings_.version.major == 0 &&
     config_settings_.version.minor == 0 &&
     config_settings_.version.micro == 0) {
    config_settings_.amplifier_gain = amplifier_gain_;
    config_settings_.version.micro = 1;
    save_config();
  }

  if (config_settings_.version.major == 0 &&
     config_settings_.version.minor == 0 &&
     config_settings_.version.micro == 1) {
    config_settings_.switching_board_i2c_address = 0x20;
    config_settings_.version.micro = 2;
    save_config();
  }

  if (config_settings_.version.major == 0 &&
     config_settings_.version.minor == 0 &&
     config_settings_.version.micro == 2) {
    config_settings_.voltage_tolerance = default_voltage_tolerance;
    config_settings_.version.micro = 3;
    save_config();
  }

  if (config_settings_.version.major == 0 &&
     config_settings_.version.minor == 0 &&
     config_settings_.version.micro == 3) {
    config_settings_.use_antialiasing_filter = false;
    config_settings_.version.micro = 4;
    save_config();
  }

  if (config_settings_.version.major == 0 &&
     config_settings_.version.minor == 0 &&
     config_settings_.version.micro == 4) {
    config_settings_.waveform_frequency_range[0] = \
        default_min_waveform_frequency;
    config_settings_.waveform_frequency_range[1] = \
        default_max_waveform_frequency;
    config_settings_.max_waveform_voltage = default_max_waveform_voltage;
    config_settings_.version.micro = 5;
    save_config();
  }

  // If we're not at the expected version by the end of the upgrade path,
  // set everything to default values.
  if (!(config_settings_.version.major == 0 &&
     config_settings_.version.minor == 0 &&
     config_settings_.version.micro == 5) || use_defaults) {

    config_settings_.version.major = 0;
    config_settings_.version.minor = 0;
    config_settings_.version.micro = 5;

    // Versions > 1.2 use the built in 5V AREF
    #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ < 3
      config_settings_.aref = 255;
    #endif

    #if ___HARDWARE_MAJOR_VERSION___ == 1
      config_settings_.vgnd = 124;
      config_settings_.waveout_gain_1 = 112;
      config_settings_.A0_series_resistance[0] = 30e4;
      config_settings_.A0_series_resistance[1] = 3.3e5;
      config_settings_.A1_series_resistance[0] = 1e3;
      config_settings_.A1_series_resistance[1] = 1e4;
      config_settings_.A1_series_resistance[2] = 1e5;
      config_settings_.A1_series_resistance[3] = 1e6;
    #else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
      config_settings_.A0_series_resistance[0] = 20e3;
      config_settings_.A0_series_resistance[1] = 200e3;
      config_settings_.A0_series_resistance[2] = 2e6;
      config_settings_.A1_series_resistance[0] = 2e2;
      config_settings_.A1_series_resistance[1] = 2e3;
      config_settings_.A1_series_resistance[2] = 2e4;
      config_settings_.A1_series_resistance[3] = 2e5;
      config_settings_.A1_series_resistance[4] = 2e6;
      config_settings_.A0_series_capacitance[2] = 50e-12;
      config_settings_.A1_series_capacitance[4] = 50e-12;
      config_settings_.signal_generator_board_i2c_address = 10;
    #endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else
    config_settings_.A0_series_capacitance[0] = 0;
    config_settings_.A0_series_capacitance[1] = 0;
    config_settings_.A1_series_capacitance[0] = 50e-12;
    config_settings_.A1_series_capacitance[1] = 50e-12;
    config_settings_.A1_series_capacitance[2] = 50e-12;
    config_settings_.A1_series_capacitance[3] = 50e-12;
    config_settings_.amplifier_gain = amplifier_gain_;
    config_settings_.switching_board_i2c_address = 0x20;
    config_settings_.voltage_tolerance = default_voltage_tolerance;
    config_settings_.use_antialiasing_filter = false;
    config_settings_.waveform_frequency_range[0] = \
        default_min_waveform_frequency;
    config_settings_.waveform_frequency_range[1] = \
        default_max_waveform_frequency;
    config_settings_.max_waveform_voltage = default_max_waveform_voltage;
    save_config();
  }

  // An amplifier gain <= 0 means that we should be doing an automatic
  // adjustment of the gain based on measurements from the amplifier output.
  if (config_settings_.amplifier_gain <= 0) {
    auto_adjust_amplifier_gain_ = true;
  } else {
    amplifier_gain_ = config_settings_.amplifier_gain;
    auto_adjust_amplifier_gain_ = false;
  }

  // if we're using the anti-aliasing filter, the feedback channel is A2
  if (config_settings_.use_antialiasing_filter) {
    feedback_controller_.begin(this, 0, 2);
  } else {
    feedback_controller_.begin(this, 0, 1);
  }
}

void DMFControlBoard::save_config() {
  uint8_t* p = (uint8_t * )&config_settings_;
  for (uint16_t i = 0; i < sizeof(ConfigSettings); i++) {
    RemoteObject::persistent_write(PERSISTENT_CONFIG_SETTINGS + i, p[i]);
  }
  /* Call `load_config` to refresh the `ConfigSettings` struct with the new
   * value written to persistent storage _(e.g., EEPROM)_. */
  load_config();
}

void DMFControlBoard::set_amplifier_gain(const float gain) {
  amplifier_gain_ = gain;
  // Update output voltage (accounting for new amplifier gain)
  // Don't wait for a response to keep things fast.
  set_waveform_voltage(waveform_voltage_, false);
}

uint8_t DMFControlBoard::set_waveform_voltage(const float output_vrms,
                                              const bool wait_for_reply) {
  uint8_t return_code = RETURN_OK;
#if ___HARDWARE_MAJOR_VERSION___==1
  float step = output_vrms / amplifier_gain_ * 2 * sqrt(2) / 4 * 255;
  if (output_vrms < 0 || \
      output_vrms > config_settings_.max_waveform_voltage || step > 255) {
    return_code = RETURN_BAD_VALUE;
  } else {
    waveform_voltage_ = output_vrms;
    set_pot(POT_INDEX_WAVEOUT_GAIN_2_, step);
  }
#else  // #if ___HARDWARE_MAJOR_VERSION___==1
  if (output_vrms <= config_settings_.max_waveform_voltage) {
    float vrms = output_vrms / amplifier_gain_;
    uint8_t data[5];
    data[0] = CMD_SET_WAVEFORM_VOLTAGE;
    memcpy(&data[1], &vrms, sizeof(float));
    i2c_write(config_settings_.signal_generator_board_i2c_address,
              data, 5);
    if (wait_for_reply) {
      delay(I2C_DELAY);
      Wire.requestFrom(config_settings_.signal_generator_board_i2c_address,
                       (uint8_t)1);
      if (Wire.available()) {
        uint8_t n_bytes_to_read = Wire.read();
        if (n_bytes_to_read==1) {
          uint8_t n_bytes_read = 0;
          n_bytes_read += i2c_read(
            config_settings_.signal_generator_board_i2c_address,
            (uint8_t*)&return_code,
            sizeof(return_code));
        }
      }
    }
  } else {
    return_code = RETURN_BAD_VALUE;
  }
  if (return_code==RETURN_OK) {
    waveform_voltage_ = output_vrms;
  }
#endif  // #if ___HARDWARE_MAJOR_VERSION___==1 / #else
  return return_code;
}

uint8_t DMFControlBoard::set_waveform_frequency(const float frequency) {
  uint8_t return_code;
  // Check that the waveform frequency is within the valid range (allow a 1%
  // tolerance to ensure that we don't accidentally throw exceptions due to
  // floating point error).
  if (frequency >= \
    config_settings_.waveform_frequency_range[0] * 0.99 && \
    frequency <= config_settings_.waveform_frequency_range[1] * 1.01) {
  #if ___HARDWARE_MAJOR_VERSION___ == 1
    // the frequency of the LTC6904 oscillator needs to be set to 50x
    // the fundamental frequency
    float freq = frequency * 50;
    // valid frequencies are 1kHz to 68MHz
    if (freq < 1e3 || freq > 68e6) {
      return_code = RETURN_BAD_VALUE;
    } else {
      uint8_t oct = 3.322 * log(freq / 1039) / log(10);
      uint16_t dac = round(2048 - (2078 * pow(2, 10 + oct)) / freq);
      uint8_t cnf = 2; // CLK on, /CLK off
      // msb = OCT3 OCT2 OCT1 OCT0 DAC9 DAC8 DAC7 DAC6
      uint8_t msb = (oct << 4) | (dac >> 6);
      // lsb =  DAC5 DAC4 DAC3 DAC2 DAC1 DAC0 CNF1 CNF0
      uint8_t lsb = (dac << 2) | cnf;
      Wire.beginTransmission(LTC6904_);
      Wire.write(msb);
      Wire.write(lsb);
      Wire.endTransmission();     // stop transmitting
      return_code = RETURN_OK;
    }
  #else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
    uint8_t data[5];
    data[0] = CMD_SET_WAVEFORM_FREQUENCY;
    memcpy(&data[1], &frequency, sizeof(float));
    i2c_write(config_settings_.signal_generator_board_i2c_address,
              data, 5);
    delay(I2C_DELAY);
    Wire.requestFrom(config_settings_.signal_generator_board_i2c_address,
                     (uint8_t)1);
    if (Wire.available()) {
      uint8_t n_bytes_to_read = Wire.read();
      if (n_bytes_to_read == 1) {
        uint8_t n_bytes_read = 0;
        n_bytes_read += i2c_read(
          config_settings_.signal_generator_board_i2c_address,
          (uint8_t * )&return_code,
          sizeof(return_code));
      }
    }
  #endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else
  } else {
    return_code = RETURN_BAD_VALUE;
  }
  if (return_code == RETURN_OK) {
    // if we get here, the frequency was successfully updated
    waveform_frequency_ = frequency;
  }
  return return_code;
}

#else   // #ifdef AVR
///////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the PC.
//
///////////////////////////////////////////////////////////////////////////////

uint16_t DMFControlBoard::number_of_channels() {
  return send_read_command<uint16_t>(CMD_GET_NUMBER_OF_CHANNELS,
                                     "number_of_channels()");
}

vector<uint8_t> DMFControlBoard::state_of_all_channels() {
  const char* function_name = "state_of_all_channels()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_GET_STATE_OF_ALL_CHANNELS) == RETURN_OK) {
    log_message("CMD_GET_STATE_OF_ALL_CHANNELS", function_name);
    std::vector < uint8_t> state_of_channels;
    for (int i = 0; i < payload_length(); i++) {
      state_of_channels.push_back(read_uint8());
      log_message(str(format("state_of_channels_[%d]=%d") % i %
        state_of_channels[i]).c_str(), function_name);
    }
    return state_of_channels;
  }
  return std::vector<uint8_t>(); // return an empty vector
};

uint8_t DMFControlBoard::state_of_channel(const uint16_t channel) {
  serialize(&channel, sizeof(channel));
  return send_read_command<uint8_t>(CMD_GET_STATE_OF_CHANNEL,
                                    "state_of_channel()");
};

uint8_t DMFControlBoard::series_resistor_index(const uint8_t channel) {
  serialize(&channel, sizeof(channel));
  return send_read_command<uint8_t>(CMD_GET_SERIES_RESISTOR_INDEX,
                                    "series_resistor_index()");
}

float DMFControlBoard::series_resistance(const uint8_t channel) {
  serialize(&channel, sizeof(channel));
  return send_read_command<float>(CMD_GET_SERIES_RESISTANCE,
                                  "series_resistance()");
}

float DMFControlBoard::series_capacitance(const uint8_t channel) {
  serialize(&channel, sizeof(channel));
  return send_read_command<float>(CMD_GET_SERIES_CAPACITANCE,
                                  "series_capacitance()");
}

float DMFControlBoard::amplifier_gain() {
    return send_read_command<float>(CMD_GET_AMPLIFIER_GAIN,
                                    "amplifier_gain()");
}

bool DMFControlBoard::auto_adjust_amplifier_gain() {
    uint8_t result = send_read_command<uint8_t>(
        CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN, "auto_adjust_amplifier_gain()");
    return result > 0;
}

std::string DMFControlBoard::waveform() {
    const char* function_name = "waveform()";
    uint8_t waveform_type = send_read_command<uint8_t>(CMD_GET_WAVEFORM,
                                                       function_name);
    std::string waveform_str;
    if (waveform_type == SINE || waveform_type == SQUARE) {
        if (waveform_type == SINE) {
          waveform_str = "SINE";
        } else if (waveform_type == SQUARE) {
          waveform_str = "SQUARE";
        }
        log_message(str(format("waveform=%s") % waveform_str).c_str(),
                   function_name);
        return waveform_str;
    } else {
        return_code_ = RETURN_BAD_VALUE;
        log_message("CMD_GET_WAVEFORM, Bad value", function_name);
    }
}

float DMFControlBoard::waveform_voltage() {
    return send_read_command<float>(CMD_GET_WAVEFORM_VOLTAGE,
                                    "waveform_voltage()");
}

float DMFControlBoard::waveform_frequency() {
    return send_read_command<float>(CMD_GET_WAVEFORM_FREQUENCY,
                                    "waveform_frequency()");
}

uint8_t DMFControlBoard::power_supply_pin() {
    return send_read_command<uint8_t>(CMD_GET_POWER_SUPPLY_PIN,
                                      "power_supply_pin()");
}

bool DMFControlBoard::watchdog_state() {
    return send_read_command<uint8_t>(CMD_GET_WATCHDOG_STATE,
                                      "watchdog_state()");
}

bool DMFControlBoard::watchdog_enabled() {
    return send_read_command<uint8_t>(CMD_GET_WATCHDOG_ENABLED,
                                      "watchdog_enabled()");
}

bool DMFControlBoard::atx_power_state() {
    return send_read_command<uint8_t>(CMD_GET_ATX_POWER_STATE,
                                      "atx_power_state()");
}

uint8_t DMFControlBoard::set_watchdog_state(bool state) {
  return send_set_command(CMD_SET_WATCHDOG_STATE, "set_watchdog_state()",
                          (uint8_t)state);
}

uint8_t DMFControlBoard::set_watchdog_enabled(bool on) {
    return send_set_command(CMD_SET_WATCHDOG_ENABLED, "set_watchdog_enabled()",
                            (uint8_t)on);
}

uint8_t DMFControlBoard::set_atx_power_state(bool state) {
    return send_set_command(CMD_SET_ATX_POWER_STATE, "set_atx_power_state()",
                            (uint8_t)state);
}

uint8_t DMFControlBoard::set_series_resistor_index(const uint8_t channel,
                                                   const uint8_t index) {
    serialize(&channel, sizeof(channel));
    return send_set_command(CMD_SET_SERIES_RESISTOR_INDEX,
                            "set_series_resistor_index()", index);
}

uint8_t DMFControlBoard::set_series_resistance(const uint8_t channel,
                                               float resistance) {
    serialize(&channel, sizeof(channel));
    return send_set_command(CMD_SET_SERIES_RESISTANCE,
                            "set_series_resistance()", resistance);
}

uint8_t DMFControlBoard::set_series_capacitance(const uint8_t channel,
                                                float capacitance) {
    serialize(&channel, sizeof(channel));
    return send_set_command(CMD_SET_SERIES_CAPACITANCE,
                            "set_series_capacitance()", capacitance);
}

uint8_t DMFControlBoard::set_amplifier_gain(float gain) {
    return send_set_command(CMD_SET_AMPLIFIER_GAIN, "set_amplifier_gain()",
                            gain);
}

uint8_t DMFControlBoard::set_auto_adjust_amplifier_gain(bool on) {
    return send_set_command(CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN,
                            "set_auto_adjust_amplifier_gain()",
                            (uint8_t)on);
}

uint8_t DMFControlBoard::set_state_of_all_channels(const vector <uint8_t>
                                                   state) {
  const char* function_name = "set_state_of_all_channels()";
  log_separator();
  log_message("send command", function_name);
  serialize(&state[0],state.size() * sizeof(uint8_t));
  if (send_command(CMD_SET_STATE_OF_ALL_CHANNELS) == RETURN_OK) {
    log_message("CMD_SET_STATE_OF_ALL_CHANNELS", function_name);
    log_message("all channels set successfully", function_name);
  }
  return return_code();
}

uint8_t DMFControlBoard::set_state_of_channel(const uint16_t channel,
                                              const uint8_t state) {
    serialize(&channel, sizeof(channel));
    return send_set_command(CMD_SET_STATE_OF_CHANNEL, "set_state_of_channel()",
                            state);
}

uint8_t DMFControlBoard::set_waveform(bool waveform) {
    return send_set_command(CMD_SET_WAVEFORM, "set_waveform()", waveform);
}

uint8_t DMFControlBoard::set_waveform_voltage(const float v_rms){
    return send_set_command(CMD_SET_WAVEFORM_VOLTAGE, "set_waveform_voltage()",
                            v_rms);
}

uint8_t DMFControlBoard::set_waveform_frequency(const float freq_hz) {
    return send_set_command(CMD_SET_WAVEFORM_FREQUENCY,
                            "set_waveform_frequency()", freq_hz);
}

std::vector <float> DMFControlBoard::measure_impedance(
                                           float sampling_window_ms,
                                           uint16_t n_sampling_windows,
                                           float delay_between_windows_ms,
                                           bool interleave_samples,
                                           bool rms,
                                           const std::vector<uint8_t> state) {
  float _sampling_rate = sampling_rate();
  measure_impedance_non_blocking(sampling_window_ms,
                                 n_sampling_windows,
                                 delay_between_windows_ms,
                                 interleave_samples,
                                 rms,
                                 state);

  boost::this_thread::sleep(boost::posix_time::milliseconds(
      (sampling_window_ms + delay_between_windows_ms) * n_sampling_windows));
  return get_impedance_data();
}

void DMFControlBoard::measure_impedance_non_blocking(
                                            float sampling_window_ms,
                                            uint16_t n_sampling_windows,
                                            float delay_between_windows_ms,
                                            bool interleave_samples,
                                            bool rms,
                                            const std::vector<uint8_t> state) {
  const char* function_name = "measure_impedance_non_blocking()";
  log_separator();
  log_message("send command", function_name);
  // if we get this far, everything is ok
  serialize(&sampling_window_ms, sizeof(sampling_window_ms));
  serialize(&n_sampling_windows, sizeof(n_sampling_windows));
  serialize(&delay_between_windows_ms, sizeof(delay_between_windows_ms));

  // set impedance options
  // IMPOPT: - - - - - - RMS INTLV
  uint8_t options = (interleave_samples << INTLV) + \
    (rms << RMS);

  serialize(&options, sizeof(options));
  serialize(&state[0], state.size() * sizeof(uint8_t));
  send_non_blocking_command(CMD_MEASURE_IMPEDANCE);
}

std::vector<float> DMFControlBoard::get_impedance_data() {
  const char* function_name = "get_impedance_data()";
  if (validate_reply(CMD_MEASURE_IMPEDANCE) == RETURN_OK) {
    uint16_t n_samples = (payload_length() - 4 * sizeof(float)) / \
        (2 * sizeof(int16_t) + 2 * sizeof(int8_t));
    log_message(str(format("Read %d impedance samples") % n_samples).c_str(),
                function_name);
    std::vector <float> impedance_buffer(4 * n_samples + 4);
    for (uint16_t i = 0; i < n_samples; i++) {
      impedance_buffer[4 * i] = read_uint16();     // V_hv
      impedance_buffer[4 * i + 1] = read_int8();  // hv_resistor
      impedance_buffer[4 * i + 2] = read_uint16(); // V_fb
      impedance_buffer[4 * i + 3] = read_int8();  // fb_resistor
    }
    for (uint16_t i = 0; i < 4; i++) {
      impedance_buffer[4*n_samples + i] = read_float();
    }
    log_message(str(format("payload_length()=%d") % payload_length()).c_str(),
                function_name);
    log_message(str(format("bytes_read() - payload_length()=%d")
                % (bytes_read() - payload_length())).c_str(), function_name);
    return impedance_buffer;
  }
  return std::vector<float>(); // return an empty vector
}

uint8_t DMFControlBoard::load_config(bool use_defaults) {
	const char* function_name = "load_config()";
  log_separator();
  uint8_t data = use_defaults;
  serialize(&data, sizeof(data));
  if (send_command(CMD_LOAD_CONFIG) == RETURN_OK) {
    log_message("CMD_LOAD_CONFIG", function_name);
    log_message("config loaded successfully", function_name);
  }
  return return_code();
}

#endif // defined(AVR) || defined(__SAM3X8E__)
