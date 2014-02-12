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
#include "dmf_control_board.h"
#ifndef AVR
  #include <boost/format.hpp>
  using namespace std;
  using boost::format;
#else
  #include "Arduino.h"
  #include <Wire.h>
  #include <OneWire.h>
  #include <SPI.h>
  #include <EEPROM.h>
  #include <math.h>

  // macros for converting hardware major/minor versions to a version string
  #define STR_VALUE(arg) #arg
  #define DEFINE_TO_STRING(name) STR_VALUE(name)
  #define ___HARDWARE_VERSION___ \
   DEFINE_TO_STRING(___HARDWARE_MAJOR_VERSION___.___HARDWARE_MINOR_VERSION___)
#endif

#ifdef AVR
  const float DmfControlBoard::SAMPLING_RATES_[] = { 8908, 16611, 29253, 47458,
                                                     68191, 90293, 105263 };
  const char DmfControlBoard::PROTOCOL_NAME_[] = "DMF Control Protocol";
  const char DmfControlBoard::PROTOCOL_VERSION_[] = "0.1";

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
  const char DmfControlBoard::CSV_INDENT_[] = ",,,,,,,,";
#endif

const char DmfControlBoard::NAME_[] = "Arduino DMF Controller";
const char DmfControlBoard::MANUFACTURER_[] = "Wheeler Microfluidics Lab";
const char DmfControlBoard::SOFTWARE_VERSION_[] = ___SOFTWARE_VERSION___;
const char DmfControlBoard::URL_[] =
    "http://microfluidics.utoronto.ca/dmf_control_board";

DmfControlBoard::DmfControlBoard()
  : RemoteObject(BAUD_RATE,true
#ifndef AVR
                   ,"DmfControlBoard" //used for logging
#endif
                   ) {
}

DmfControlBoard::~DmfControlBoard() {
}

uint8_t DmfControlBoard::ProcessCommand(uint8_t cmd) {
#ifndef AVR
  const char* function_name = "ProcessCommand()";
  LogMessage(str(format("command=0x%0X (%d)") % cmd % cmd).c_str(),
             function_name);
#endif
  switch(cmd) {
#ifdef AVR // Commands that only the Arduino handles
    case CMD_GET_NUMBER_OF_CHANNELS:
      if (payload_length() == 0) {
        Serialize(&number_of_channels_, sizeof(number_of_channels_));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_STATE_OF_ALL_CHANNELS:
      if (payload_length()==0) {
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
                Serialize(&state, sizeof(state));
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
        UpdateAllChannels();
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_STATE_OF_CHANNEL:
      if (payload_length() == sizeof(uint16_t)) {
        uint16_t channel = ReadUint16();
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
            Serialize(&data, sizeof(data));
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
        uint16_t channel = ReadUint16();
        uint8_t state = ReadUint8();
        if (channel < number_of_channels_) {
          return_code_ = UpdateChannel(channel, state);
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
        Serialize(&waveform, sizeof(waveform));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WAVEFORM:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t waveform = ReadUint8();
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
#endif
    case CMD_GET_WAVEFORM_VOLTAGE:
      if (payload_length() == 0) {
#if ___HARDWARE_MAJOR_VERSION___ == 1
        return_code_ = RETURN_OK;
        Serialize(&waveform_voltage_, sizeof(waveform_voltage_));
#else
        i2c_write(config_settings_.signal_generator_board_i2c_address, cmd);
        delay(I2C_DELAY);
        Wire.requestFrom(config_settings_.signal_generator_board_i2c_address,
                         (uint8_t)1);
        if (Wire.available()) {
          uint8_t n_bytes_to_read = Wire.read();
          if (n_bytes_to_read == sizeof(float) + 1) {
            uint8_t data[5];
            i2c_read(config_settings_.signal_generator_board_i2c_address,
                     (uint8_t *)&data[0], 5);
            return_code_ = data[4];
            if (return_code_ == RETURN_OK) {
              memcpy(&waveform_voltage_, &data[0], sizeof(float));
              waveform_voltage_ *= amplifier_gain_;
              Serialize(&waveform_voltage_, sizeof(float));
            }
          }
        }
#endif
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WAVEFORM_VOLTAGE:
      if (payload_length() == sizeof(float)) {
        return_code_ = SetWaveformVoltage(ReadFloat());
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_WAVEFORM_FREQUENCY:
      if (payload_length() == 0) {
#if ___HARDWARE_MAJOR_VERSION___ == 1
        return_code_ = RETURN_OK;
        Serialize(&waveform_frequency_, sizeof(waveform_frequency_));
#else
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
                Serialize(&waveform_frequency_, sizeof(float));
              }
            }
          }
#endif
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WAVEFORM_FREQUENCY:
      if (payload_length() == sizeof(float)) {
#if ___HARDWARE_MAJOR_VERSION___ == 1
        // the frequency of the LTC6904 oscillator needs to be set to 50x
        // the fundamental frequency
        waveform_frequency_ = ReadFloat();
        float freq = waveform_frequency_ * 50;
        // valid frequencies are 1kHz to 68MHz
        if (freq < 1e3 || freq > 68e6) {
          return_code_ = RETURN_BAD_VALUE;
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
          return_code_ = RETURN_OK;
        }
#else
        waveform_frequency_ = ReadFloat();
        uint8_t data[5];
        data[0] = cmd;
        memcpy(&data[1], &waveform_frequency_, sizeof(float));
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
              (uint8_t * )&return_code_,
              sizeof(return_code_));
          }
        }
#endif
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_SAMPLING_RATE:
      if (payload_length() == 0) {
        Serialize(&SAMPLING_RATES_[sampling_rate_index_], sizeof(float));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SAMPLING_RATE:
      if (payload_length() == sizeof(uint8_t)) {
        return_code_ = SetAdcPrescaler(ReadUint8());
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_SERIES_RESISTOR_INDEX:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t channel = ReadUint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            Serialize(&A0_series_resistor_index_,
                      sizeof(A0_series_resistor_index_));
            break;
          case 1:
            Serialize(&A1_series_resistor_index_,
                      sizeof(A1_series_resistor_index_));
            break;
          default:
            return_code_ = RETURN_BAD_INDEX;
            break;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SERIES_RESISTOR_INDEX:
      if (payload_length() == 2*sizeof(uint8_t)) {
        uint8_t channel = ReadUint8();
        return_code_ = SetSeriesResistor(channel, ReadUint8());
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_SERIES_RESISTANCE:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t channel = ReadUint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            Serialize(&config_settings_.A0_series_resistance
                      [A0_series_resistor_index_], sizeof(float));
            break;
          case 1:
            Serialize(&config_settings_.A1_series_resistance
                      [A1_series_resistor_index_], sizeof(float));
            break;
          default:
            return_code_ = RETURN_BAD_INDEX;
            break;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SERIES_RESISTANCE:
      if (payload_length() == sizeof(uint8_t) + sizeof(float)) {
        uint8_t channel = ReadUint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            config_settings_.A0_series_resistance[A0_series_resistor_index_] =
                ReadFloat();
            SaveConfig();
            break;
          case 1:
            config_settings_.A1_series_resistance[A1_series_resistor_index_] =
                ReadFloat();
            SaveConfig();
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
        uint8_t channel = ReadUint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            Serialize(&config_settings_.A0_series_capacitance
                      [A0_series_resistor_index_], sizeof(float));
            break;
          case 1:
            Serialize(&config_settings_.A1_series_capacitance
                      [A1_series_resistor_index_], sizeof(float));
            break;
          default:
            return_code_ = RETURN_BAD_INDEX;
            break;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SERIES_CAPACITANCE:
      if (payload_length() == sizeof(uint8_t) + sizeof(float)) {
        uint8_t channel = ReadUint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            config_settings_.A0_series_capacitance[A0_series_resistor_index_] =
                ReadFloat();
            SaveConfig();
            break;
          case 1:
            config_settings_.A1_series_capacitance[A1_series_resistor_index_] =
                ReadFloat();
            SaveConfig();
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
        Serialize(&amplifier_gain_, sizeof(amplifier_gain_));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_AMPLIFIER_GAIN:
      if (payload_length() == sizeof(float)) {
        float value = ReadFloat();
        if (value > 0) {
          if (auto_adjust_amplifier_gain_) {
            amplifier_gain_ = value;
          } else {
            config_settings_.amplifier_gain = value;
            auto_adjust_amplifier_gain_ = false;
            SaveConfig();
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
        Serialize(&value, sizeof(value));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t value = ReadUint8();
        if (value > 0) {
          config_settings_.amplifier_gain = 0;
        } else {
          config_settings_.amplifier_gain = amplifier_gain_;
        }
        SaveConfig();
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_MEASURE_IMPEDANCE:
      if (payload_length() < 3*sizeof(uint16_t)) {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      } else {
        uint16_t sampling_time_ms = ReadUint16();
        uint16_t n_samples = ReadUint16();
        uint16_t delay_between_samples_ms = ReadUint16();

        // only collect enough samples to fill the maximum payload length
        uint16_t max_samples = MAX_PAYLOAD_LENGTH /
                               (2 * sizeof(int8_t) + 2 * sizeof(int16_t));
        if (n_samples > max_samples) {
          n_samples = max_samples;
        }

        if (payload_length() == 3 * sizeof(uint16_t) || (payload_length() == 3 *
                                                        sizeof(uint16_t) +
                                                        number_of_channels_ *
                                                        sizeof(uint8_t))) {
          return_code_ = RETURN_OK;

          uint8_t original_A0_index = A0_series_resistor_index_;
          uint8_t original_A1_index = A1_series_resistor_index_;

          // set the resistors to their highest values
          SetSeriesResistor(0, sizeof(config_settings_.A0_series_resistance) /
                            sizeof(float) - 1);
          SetSeriesResistor(1, sizeof(config_settings_.A1_series_resistance) /
                            sizeof(float) - 1);

          // update the channels (if they were included in the packet)
          if (payload_length() == 3 * sizeof(uint16_t) + number_of_channels_ *
              sizeof(uint8_t)) {
            UpdateAllChannels();
          }

          // sample the feedback voltage
          for (uint16_t i = 0; i < n_samples; i++) {
            uint16_t hv_max = 0;
            uint16_t hv_min = 1023;
            uint16_t hv = 0;
            int16_t hv_pk_pk;
            int8_t hv_resistor = A0_series_resistor_index_;
            uint16_t fb_max = 0;
            uint16_t fb_min = 1023;
            uint16_t fb = 0;
            int16_t fb_pk_pk;
            int8_t fb_resistor = A1_series_resistor_index_;
            uint32_t t_sample = millis();

            while(millis() - t_sample < sampling_time_ms) {
              // hv_resistor == -1 if the smallest series resistor becomes
              // saturated
              if (hv_resistor != -1) {
                hv = analogRead(0);
                // if the ADC is saturated, use a smaller resistor
                // and reset the peak
                if (hv > 820) {
                  if (A0_series_resistor_index_ > 0) {
                    SetSeriesResistor(0, A0_series_resistor_index_ - 1);
                    hv_max = 0;
                    hv_min = 1023;
                  } else {
                    hv_resistor = -1;
                  }
                  continue;
                }
                if (hv > hv_max) {
                  hv_max = hv;
                }
                if (hv < hv_min) {
                  hv_min = hv;
                }
              }

              // hv_resistor == -1 if the smallest series resistor becomes
              // saturated
              if (fb_resistor != -1) {
                fb = analogRead(1);

                // if the ADC is saturated, use a smaller resistor
                // and reset the peak
                if (fb > 820) {
                  if (A1_series_resistor_index_ > 0) {
                    SetSeriesResistor(1, A1_series_resistor_index_ - 1);
                    fb_max = 0;
                    fb_min = 1023;
                  } else {
                    fb_resistor = -1;
                  }
                  continue;
                }
                if (fb > fb_max) {
                  fb_max = fb;
                }
                if (fb < fb_min) {
                  fb_min = fb;
                }
              }
            }

            hv_pk_pk = hv_max - hv_min;
            fb_pk_pk = fb_max - fb_min;

            // if we didn't get a valid feedback sample during the sampling
            // time, return -1 as the index
            if (fb_max == 0 || fb_min == 1023 && fb_resistor != -1) {
              fb_resistor = -1;
            } else {
              fb_resistor = A1_series_resistor_index_;
            }

            // if we didn't get a valid high voltage sample during the
            // sampling time, return -1 as the index
            if (hv_max == 0 || hv_min == 1023 && hv_resistor != -1) {
              hv_resistor = -1;
            } else {
              // adjust amplifier gain (only if the hv resistor is the same
              // as on the previous reading; otherwise it may not have had
              // enough time to get a good reading)
              if (auto_adjust_amplifier_gain_ && waveform_voltage_ > 0 && i > 0
                  && hv_resistor == A0_series_resistor_index_) {
                float R = config_settings_.A0_series_resistance
                                           [A0_series_resistor_index_];
                float C = config_settings_.A0_series_capacitance
                                           [A0_series_resistor_index_];
                float measured_voltage, set_voltage;
#if ___HARDWARE_MAJOR_VERSION___ == 1
                float V_fb;
                if (fb_resistor == - 1 || fb_pk_pk < 0) {
                  V_fb = 0;
                } else {
                  V_fb = fb_pk_pk * 5.0 / 1023 / sqrt(2) / 2;
                }
                measured_voltage = (hv_pk_pk * 5.0 / 1023.0 // measured Vrms /
                                    / sqrt(2) / 2) /
                                   (1 / sqrt(pow(10e6 / R   // transfer
                                                 + 1, 2)    // function
                                             + pow(10e6 * C * 2 * M_PI *
                                                   waveform_frequency_, 2)));
                set_voltage = waveform_voltage_ + V_fb;
#else
                measured_voltage = (hv_pk_pk * 5.0 / 1023.0  // measured Vrms /
                                    / sqrt(2) / 2) /
                                   (1 / sqrt(pow(10e6 / R,   // transfer
                                                 2) +        // function
                                             pow(10e6 * C * 2 * M_PI *
                                                 waveform_frequency_, 2)));
                set_voltage = waveform_voltage_;
#endif
                // if we're outside of the voltage tolerance, update the gain
                if (abs(measured_voltage - set_voltage) >
                   config_settings_.voltage_tolerance) {
                  amplifier_gain_ *= measured_voltage / set_voltage;

                  // enforce minimum gain of 1 because if gain goes to zero,
                  // it cannot be adjusted further
                  if (amplifier_gain_ < 1) {
                    amplifier_gain_ = 1;
                  }
#if ___HARDWARE_MAJOR_VERSION___ == 1
                  // update output voltage (accounting for amplifier gain and
                  // for the voltage drop across the feedback resistor)
                  SetWaveformVoltage(waveform_voltage_ + V_fb);
#else
                  // update output voltage (but don't wait for i2c response)
                  SetWaveformVoltage(waveform_voltage_, false);
#endif
                }
              }
              hv_resistor = A0_series_resistor_index_;
            }

            Serialize(&hv_pk_pk, sizeof(hv_pk_pk));
            Serialize(&hv_resistor, sizeof(hv_resistor));
            Serialize(&fb_pk_pk, sizeof(fb_pk_pk));
            Serialize(&fb_resistor, sizeof(fb_resistor));

            if (Serial.available() > 0) {
              break;
            }

            uint32_t t_delay = millis();
            while(millis() - t_delay < delay_between_samples_ms) {}
          }

          // set the resistors back to their original states
          SetSeriesResistor(0, original_A0_index);
          SetSeriesResistor(1, original_A1_index);
        } else {
          return_code_ = RETURN_BAD_PACKET_SIZE;
        }
      }
      break;
    case CMD_RESET_CONFIG_TO_DEFAULTS:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        LoadConfig(true);
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#endif
  }
  RemoteObject::ProcessCommand(cmd);
#ifndef AVR
  if (return_code_ == RETURN_UNKNOWN_COMMAND) {
    LogError("Unrecognized command", function_name);
  }
#endif
  return return_code_;
}

#ifdef AVR
///////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the Arduino.
//
///////////////////////////////////////////////////////////////////////////////

void DmfControlBoard::begin() {
  RemoteObject::begin();

  // Versions > 1.2 use the built in 5V AREF (default)
  #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ < 3
    analogReference(EXTERNAL);
  #endif

  #if ___HARDWARE_MAJOR_VERSION___ == 1
    pinMode(AD5204_SLAVE_SELECT_PIN_, OUTPUT);
    pinMode(A0_SERIES_RESISTOR_0_, OUTPUT);
    pinMode(A1_SERIES_RESISTOR_0_, OUTPUT);
    pinMode(A1_SERIES_RESISTOR_1_, OUTPUT);
    pinMode(A1_SERIES_RESISTOR_2_, OUTPUT);
    pinMode(WAVEFORM_SELECT_, OUTPUT);
  #else
    pinMode(A0_SERIES_RESISTOR_0_, OUTPUT);
    pinMode(A0_SERIES_RESISTOR_1_, OUTPUT);
    pinMode(A1_SERIES_RESISTOR_0_, OUTPUT);
    pinMode(A1_SERIES_RESISTOR_1_, OUTPUT);
    pinMode(A1_SERIES_RESISTOR_2_, OUTPUT);
    pinMode(A1_SERIES_RESISTOR_3_, OUTPUT);
  #endif

  // versions > 1.1 need to pull a pin low to turn on the power supply
  #if (___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ > 1) \
    || ___HARDWARE_MAJOR_VERSION___ > 1
    pinMode(PWR_SUPPLY_ON_, OUTPUT);
    digitalWrite(PWR_SUPPLY_ON_, LOW);

    // wait for the power supply to turn on
    delay(500);
  #endif

  Serial.begin(DmfControlBoard::BAUD_RATE);

  Serial.print(name());
  Serial.print(" v");
  Serial.println(hardware_version());
  Serial.print("Firmware version: ");
  Serial.println(software_version());

  i2c_scan();

  #if ___HARDWARE_MAJOR_VERSION___ == 1
    // set waveform (SINE=0, SQUARE=1)
    digitalWrite(WAVEFORM_SELECT_, SINE);
  #endif

  // default amplifier gain
  amplifier_gain_ = 100;

  LoadConfig();
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
    Serial.print("A0_series_resistance[0]=");
    Serial.println(config_settings_.A0_series_resistance[0]);
    Serial.print("A0_series_resistance[1]=");
    Serial.println(config_settings_.A0_series_resistance[1]);
    Serial.print("A0_series_capacitance[0]=");
    printlne(config_settings_.A0_series_capacitance[0]);
    Serial.print("A0_series_capacitance[1]=");
    printlne(config_settings_.A0_series_capacitance[1]);
    Serial.print("A1_series_resistance[0]=");
    Serial.println(config_settings_.A1_series_resistance[0]);
    Serial.print("A1_series_resistance[1]=");
    Serial.println(config_settings_.A1_series_resistance[1]);
    Serial.print("A1_series_resistance[2]=");
    Serial.println(config_settings_.A1_series_resistance[2]);
    Serial.print("A1_series_resistance[3]=");
    Serial.println(config_settings_.A1_series_resistance[3]);
    Serial.print("A1_series_capacitance[0]=");
    printlne(config_settings_.A1_series_capacitance[0]);
    Serial.print("A1_series_capacitance[1]=");
    printlne(config_settings_.A1_series_capacitance[1]);
    Serial.print("A1_series_capacitance[2]=");
    printlne(config_settings_.A1_series_capacitance[2]);
    Serial.print("A1_series_capacitance[3]=");
    printlne(config_settings_.A1_series_capacitance[3]);
  #else
    Serial.print("A0_series_resistance[0]=");
    Serial.println(config_settings_.A0_series_resistance[0]);
    Serial.print("A0_series_resistance[1]=");
    Serial.println(config_settings_.A0_series_resistance[1]);
    Serial.print("A0_series_resistance[2]=");
    Serial.println(config_settings_.A0_series_resistance[2]);
    Serial.print("A0_series_capacitance[0]=");
    printlne(config_settings_.A0_series_capacitance[0]);
    Serial.print("A0_series_capacitance[1]=");
    printlne(config_settings_.A0_series_capacitance[1]);
    Serial.print("A0_series_capacitance[2]=");
    printlne(config_settings_.A0_series_capacitance[2]);
    Serial.print("A1_series_resistance[0]=");
    Serial.println(config_settings_.A1_series_resistance[0]);
    Serial.print("A1_series_resistance[1]=");
    Serial.println(config_settings_.A1_series_resistance[1]);
    Serial.print("A1_series_resistance[2]=");
    Serial.println(config_settings_.A1_series_resistance[2]);
    Serial.print("A1_series_resistance[3]=");
    Serial.println(config_settings_.A1_series_resistance[3]);
    Serial.print("A1_series_resistance[4]=");
    Serial.println(config_settings_.A1_series_resistance[4]);
    Serial.print("A1_series_capacitance[0]=");
    printlne(config_settings_.A1_series_capacitance[0]);
    Serial.print("A1_series_capacitance[1]=");
    printlne(config_settings_.A1_series_capacitance[1]);
    Serial.print("A1_series_capacitance[2]=");
    printlne(config_settings_.A1_series_capacitance[2]);
    Serial.print("A1_series_capacitance[3]=");
    printlne(config_settings_.A1_series_capacitance[3]);
    Serial.print("A1_series_capacitance[4]=");
    printlne(config_settings_.A1_series_capacitance[4]);
    Serial.print("signal_generator_board_i2c_address=");
    Serial.println(config_settings_.signal_generator_board_i2c_address, DEC);
  #endif

  Serial.print("switching_board_i2c_address=");
  Serial.println(config_settings_.switching_board_i2c_address, DEC);
  Serial.print("amplifier_gain=");
  Serial.println(config_settings_.amplifier_gain);
  Serial.print("voltage_tolerance=");
  Serial.println(config_settings_.voltage_tolerance);

  // Check how many switching boards are connected.  Each additional board's
  // address must equal the previous boards address +1 to be valid.
  number_of_channels_ = 0;
  for (uint8_t chip = 0; chip < 8; chip++) {
    Wire.beginTransmission(
      config_settings_.switching_board_i2c_address + chip);
    Wire.write(PCA9505_CONFIG_IO_REGISTER_);
    Wire.endTransmission();
    Wire.requestFrom(
      config_settings_.switching_board_i2c_address + chip,1);
    if (Wire.available()) {
      Wire.read();
      if (number_of_channels_ == 40 * chip) {
        number_of_channels_ = 40 * (chip + 1);
      }
      Serial.print("HV board ");
      Serial.print((int)chip);
      Serial.println(" connected.");
      uint8_t data[2];
      // set all PCA0505 ports in output mode and initialize to ground
      for (uint8_t port=0; port<5; port++) {
        data[0] = PCA9505_CONFIG_IO_REGISTER_ + port;
        data[1] = 0x00;
        i2c_write(config_settings_.switching_board_i2c_address + chip,
                  data, 2);
        data[0] = PCA9505_OUTPUT_PORT_REGISTER_ + port;
        data[1] = 0xFF;
        i2c_write(config_settings_.switching_board_i2c_address + chip,
                  data, 2);
      }
    }
  }
  Serial.print(number_of_channels_);
  Serial.println(" channels available.");

  // set all digital pots

  // Versions > 1.2 use the built in 5V AREF
  #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ < 3
    SetPot(POT_INDEX_AREF_, config_settings_.aref);
  #endif
  #if ___HARDWARE_MAJOR_VERSION___ == 1
    SetPot(POT_INDEX_VGND_, config_settings_.vgnd);
    SetPot(POT_INDEX_WAVEOUT_GAIN_1_, config_settings_.waveout_gain_1);
    SetPot(POT_INDEX_WAVEOUT_GAIN_2_, 0);
  #endif

  SetSeriesResistor(0, 0);
  SetSeriesResistor(1, 0);
  SetAdcPrescaler(4);
}

const char* DmfControlBoard::hardware_version() {
  return ___HARDWARE_VERSION___;
}

void DmfControlBoard::SendSPI(uint8_t pin, uint8_t address, uint8_t data) {
  digitalWrite(pin, LOW);
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(pin, HIGH);
}

uint8_t DmfControlBoard::SetPot(uint8_t index, uint8_t value) {
  if (index>=0 && index<4) {
    SendSPI(AD5204_SLAVE_SELECT_PIN_, index, 255-value);
    return RETURN_OK;
  }
  return RETURN_BAD_INDEX;
}

uint8_t DmfControlBoard::SetAdcPrescaler(const uint8_t index) {
  uint8_t return_code = RETURN_OK;
  switch(128>>index) {
    case 128:
      ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
      break;
    case 64:
      ADCSRA |= _BV(ADPS2) | _BV(ADPS1);
      ADCSRA &= ~(_BV(ADPS0));
      break;
    case 32:
      ADCSRA |= _BV(ADPS2) | _BV(ADPS0);
      ADCSRA &= ~_BV(ADPS1);
      break;
    case 16:
      ADCSRA |= _BV(ADPS2);
      ADCSRA &= ~(_BV(ADPS1) | _BV(ADPS0));
      break;
    case 8:
      ADCSRA |= _BV(ADPS1) | _BV(ADPS0);
      ADCSRA &= ~_BV(ADPS2);
      break;
    case 4:
      ADCSRA |= _BV(ADPS1);
      ADCSRA &= ~(_BV(ADPS2) | _BV(ADPS0));
      break;
    case 2:
      ADCSRA |= _BV(ADPS0);
      ADCSRA &= ~(_BV(ADPS2) | _BV(ADPS1));
      break;
    default:
      return_code = RETURN_GENERAL_ERROR;
      break;
  }
  if (return_code==RETURN_OK){
    sampling_rate_index_ = index;
  }
  return return_code;
}


uint8_t DmfControlBoard::SetSeriesResistor(const uint8_t channel,
                                           const uint8_t index) {
  uint8_t return_code = RETURN_OK;
  if (channel==0) {
    switch(index) {
      case 0:
        digitalWrite(A0_SERIES_RESISTOR_0_, HIGH);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(A0_SERIES_RESISTOR_1_, LOW);
        #endif
        break;
      case 1:
        digitalWrite(A0_SERIES_RESISTOR_0_, LOW);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(A0_SERIES_RESISTOR_1_, HIGH);
        #endif
        break;
#if ___HARDWARE_MAJOR_VERSION___ == 2
      case 2:
        digitalWrite(A0_SERIES_RESISTOR_0_, LOW);
        digitalWrite(A0_SERIES_RESISTOR_1_, LOW);
        break;
#endif
      default:
        return_code = RETURN_BAD_INDEX;
        break;
    }
    if (return_code==RETURN_OK) {
      A0_series_resistor_index_ = index;
    }
  } else if (channel==1) {
    switch(index) {
      case 0:
        digitalWrite(A1_SERIES_RESISTOR_0_, HIGH);
        digitalWrite(A1_SERIES_RESISTOR_1_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_2_, LOW);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(A1_SERIES_RESISTOR_3_, LOW);
        #endif
        break;
      case 1:
        digitalWrite(A1_SERIES_RESISTOR_0_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_1_, HIGH);
        digitalWrite(A1_SERIES_RESISTOR_2_, LOW);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(A1_SERIES_RESISTOR_3_, LOW);
        #endif
        break;
      case 2:
        digitalWrite(A1_SERIES_RESISTOR_0_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_1_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_2_, HIGH);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(A1_SERIES_RESISTOR_3_, LOW);
        #endif
        break;
      case 3:
        digitalWrite(A1_SERIES_RESISTOR_0_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_1_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_2_, LOW);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(A1_SERIES_RESISTOR_3_, HIGH);
        #endif
        break;
      #if ___HARDWARE_MAJOR_VERSION___ == 2
      case 4:
        digitalWrite(A1_SERIES_RESISTOR_0_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_1_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_2_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_3_, LOW);
        break;
      #endif
      default:
        return_code = RETURN_BAD_INDEX;
        break;
    }
    if (return_code==RETURN_OK) {
      A1_series_resistor_index_ = index;
    }
  } else { // bad channel
    return_code = RETURN_BAD_INDEX;
  }
  // wait for signal to settle
  delayMicroseconds(200);
  return return_code;
}

// update the state of all channels
void DmfControlBoard::UpdateAllChannels() {
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
        data[1] += (ReadUint8() == 0) << i;
      }
      i2c_write(config_settings_.switching_board_i2c_address + chip,
                data, 2);
    }
  }
}

// Update the state of single channel.
// Note: Do not use this function in a loop to update all channels. If you
//       want to update all channels, use the UpdateAllChannels function
//       instead because it will be 8x more efficient.
uint8_t DmfControlBoard::UpdateChannel(const uint16_t channel,
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

DmfControlBoard::version_t DmfControlBoard::ConfigVersion() {
  version_t config_version;
  uint8_t* p = (uint8_t*)&config_version;
  for (uint16_t i = 0; i < sizeof(version_t); i++) {
    p[i] = this->persistent_read(PERSISTENT_CONFIG_SETTINGS + i);
  }
  return config_version;
}

void DmfControlBoard::LoadConfig(bool use_defaults) {
  uint8_t* p = (uint8_t*)&config_settings_;
  for (uint16_t i = 0; i < sizeof(config_settings_t); i++) {
    p[i] = this->persistent_read(PERSISTENT_CONFIG_SETTINGS + i);
  }

  float default_voltage_tolerance = 5.0;

  // Upgrade config settings if necessary
  if (config_settings_.version.major == 0 &&
     config_settings_.version.minor == 0 &&
     config_settings_.version.micro == 0) {
    config_settings_.amplifier_gain = amplifier_gain_;
    config_settings_.version.micro = 1;
    SaveConfig();
  }

  if (config_settings_.version.major == 0 &&
     config_settings_.version.minor == 0 &&
     config_settings_.version.micro == 1) {
    config_settings_.switching_board_i2c_address = 0x20;
    config_settings_.version.micro = 2;
    SaveConfig();
  }

  if (config_settings_.version.major == 0 &&
     config_settings_.version.minor == 0 &&
     config_settings_.version.micro == 2) {
    config_settings_.voltage_tolerance = default_voltage_tolerance;
    config_settings_.version.micro = 3;
    SaveConfig();
  }

  // If we're not at the expected version by the end of the upgrade path,
  // set everything to default values.
  if (!(config_settings_.version.major == 0 &&
     config_settings_.version.minor == 0 &&
     config_settings_.version.micro == 3) || use_defaults) {

    config_settings_.version.major = 0;
    config_settings_.version.minor = 0;
    config_settings_.version.micro = 3;

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
    #else
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
    #endif
    config_settings_.A0_series_capacitance[0] = 0;
    config_settings_.A0_series_capacitance[1] = 0;
    config_settings_.A1_series_capacitance[0] = 50e-12;
    config_settings_.A1_series_capacitance[1] = 50e-12;
    config_settings_.A1_series_capacitance[2] = 50e-12;
    config_settings_.A1_series_capacitance[3] = 50e-12;
    config_settings_.amplifier_gain = amplifier_gain_;
    config_settings_.switching_board_i2c_address = 0x20;
    config_settings_.voltage_tolerance = default_voltage_tolerance;
    SaveConfig();
  }

  // An amplifier gain <= 0 means that we should be doing an automatic
  // adjustment of the gain based on measurements from the amplifier output.
  if (config_settings_.amplifier_gain <= 0) {
    auto_adjust_amplifier_gain_ = true;
  } else {
    amplifier_gain_ = config_settings_.amplifier_gain;
    auto_adjust_amplifier_gain_ = false;
  }
}

void DmfControlBoard::SaveConfig() {
  uint8_t* p = (uint8_t * )&config_settings_;
  for (uint16_t i = 0; i < sizeof(config_settings_t); i++) {
    RemoteObject::persistent_write(PERSISTENT_CONFIG_SETTINGS + i, p[i]);
  }
  /* Call `LoadConfig` to refresh the `config_settings_t` struct with the new
   * value written to persistent storage _(e.g., EEPROM)_. */
  LoadConfig();
}

uint8_t DmfControlBoard::SetWaveformVoltage(const float output_vrms,
                                            const bool wait_for_reply) {
  uint8_t return_code;
#if ___HARDWARE_MAJOR_VERSION___==1
  float step = output_vrms / amplifier_gain_ * 2 * sqrt(2) / 4 * 255;
  if (output_vrms < 0 || step > 255) {
    return_code = RETURN_BAD_VALUE;
  } else {
    waveform_voltage_ = output_vrms;
    SetPot(POT_INDEX_WAVEOUT_GAIN_2_, step);
    return_code = RETURN_OK;
  }
#else
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
        if (return_code==RETURN_OK) {
          waveform_voltage_ = output_vrms;
        }
      }
    }
  } else {
    waveform_voltage_ = output_vrms;
    return_code = RETURN_OK;
  }
#endif
  return return_code;
}

void /* DEVICE */ DmfControlBoard::persistent_write(uint16_t address,
                                                    uint8_t value) {
    RemoteObject::persistent_write(address, value);
    /* Reload config from persistent storage _(e.g., EEPROM)_ to refresh
     * `config_settings_t` struct with new value. */
    LoadConfig();
}

#else
///////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the PC.
//
///////////////////////////////////////////////////////////////////////////////

uint16_t DmfControlBoard::number_of_channels() {
  const char* function_name = "number_of_channels()";
  LogSeparator();
  LogMessage("send command", function_name);
  if (SendCommand(CMD_GET_NUMBER_OF_CHANNELS) == RETURN_OK) {
    if (payload_length() == sizeof(uint16_t)) {
      uint16_t number_of_channels = ReadUint16();
      LogMessage(str(format("number_of_channels=%d") %
        number_of_channels).c_str(), function_name);
      return number_of_channels;
    } else {
      LogMessage("CMD_GET_NUMBER_OF_CHANNELS, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

vector<uint8_t> DmfControlBoard::state_of_all_channels() {
  const char* function_name = "state_of_all_channels()";
  LogSeparator();
  LogMessage("send command", function_name);
  if (SendCommand(CMD_GET_STATE_OF_ALL_CHANNELS) == RETURN_OK) {
    LogMessage("CMD_GET_STATE_OF_ALL_CHANNELS", function_name);
    std::vector < uint8_t> state_of_channels;
    for (int i = 0; i < payload_length(); i++) {
      state_of_channels.push_back(ReadUint8());
      LogMessage(str(format("state_of_channels_[%d]=%d") % i %
        state_of_channels[i]).c_str(), function_name);
    }
    return state_of_channels;
  }
  return std::vector<uint8_t>(); // return an empty vector
};

uint8_t DmfControlBoard::state_of_channel(const uint16_t channel) {
  const char* function_name = "state_of_channel()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel, sizeof(channel));
  if (SendCommand(CMD_GET_STATE_OF_CHANNEL) == RETURN_OK) {
    LogMessage("CMD_GET_STATE_OF_CHANNEL", function_name);
    if (payload_length() == sizeof(uint8_t)) {
      uint8_t state = ReadUint8();
      LogMessage(str(format("state=%d") % state).c_str(), function_name);
      return state;
    } else {
      LogError("Bad packet size", function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
};

float DmfControlBoard::sampling_rate() {
  const char* function_name = "sampling_rate()";
  LogSeparator();
  LogMessage("send command", function_name);
  if (SendCommand(CMD_GET_SAMPLING_RATE) == RETURN_OK) {
    LogMessage("CMD_GET_SAMPLING_RATE", function_name);
    if (payload_length() == sizeof(float)) {
      float sampling_rate = ReadFloat();
      LogMessage(str(format("sampling_rate_=%.1e") % sampling_rate).c_str(),
        function_name);
      return sampling_rate;
    } else {
      LogMessage("CMD_GET_SAMPLING_RATE, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

uint8_t DmfControlBoard::set_sampling_rate(const uint8_t sampling_rate) {
  const char* function_name = "set_sampling_rate()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&sampling_rate, sizeof(sampling_rate));
  if (SendCommand(CMD_SET_SAMPLING_RATE) == RETURN_OK) {
    LogMessage("CMD_SET_SAMPLING_RATE", function_name);
    LogMessage("sampling rate set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::series_resistor_index(const uint8_t channel) {
  const char* function_name = "series_resistor_index()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel, sizeof(channel));
  if (SendCommand(CMD_GET_SERIES_RESISTOR_INDEX) == RETURN_OK) {
    LogMessage("CMD_GET_SERIES_RESISTOR_INDEX", function_name);
    if (payload_length() == sizeof(uint8_t)) {
      uint8_t index = ReadUint8();
      LogMessage(str(format("series_resistor_index=%d") % index).c_str(),
        function_name);
      return index;
    } else {
      LogMessage("CMD_GET_SERIES_RESISTOR_INDEX, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

float DmfControlBoard::series_resistance(const uint8_t channel) {
  const char* function_name = "series_resistance()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel, sizeof(channel));
  if (SendCommand(CMD_GET_SERIES_RESISTANCE) == RETURN_OK) {
    LogMessage("CMD_GET_SERIES_RESISTANCE", function_name);
    if (payload_length() == sizeof(float)) {
      float series_resistance = ReadFloat();
      LogMessage(str(format("series_resistance=%.1e") %
        series_resistance).c_str(), function_name);
      return series_resistance;
    } else {
      LogMessage("CMD_GET_SERIES_RESISTANCE, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

float DmfControlBoard::series_capacitance(const uint8_t channel) {
  const char* function_name = "series_capacitance()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel, sizeof(channel));
  if (SendCommand(CMD_GET_SERIES_CAPACITANCE) == RETURN_OK) {
    LogMessage("CMD_GET_SERIES_CAPACITANCE", function_name);
    if (payload_length() == sizeof(float)) {
      float series_capacitance = ReadFloat();
      LogMessage(str(format("series_capacitance=%.1e") %
        series_capacitance).c_str(), function_name);
      return series_capacitance;
    } else {
      LogMessage("CMD_GET_SERIES_CAPACITANCE, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

float DmfControlBoard::amplifier_gain() {
  const char* function_name = "amplifier_gain()";
  LogSeparator();
  LogMessage("send command", function_name);
  if (SendCommand(CMD_GET_AMPLIFIER_GAIN) == RETURN_OK) {
    LogMessage("CMD_GET_AMPLIFIER_GAIN", function_name);
    if (payload_length() == sizeof(float)) {
      float gain = ReadFloat();
      LogMessage(str(format("amplifier_gain=%.1e") % gain).c_str(),
        function_name);
      return gain;
    } else {
      LogMessage("CMD_GET_AMPLIFIER_GAIN, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

bool DmfControlBoard::auto_adjust_amplifier_gain() {
  const char* function_name = "auto_adjust_amplifier_gain()";
  LogSeparator();
  LogMessage("send command", function_name);
  if (SendCommand(CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN) == RETURN_OK) {
    LogMessage("CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN", function_name);
    if (payload_length() == sizeof(uint8_t)) {
      uint8_t value = ReadUint8();
      LogMessage(str(format("auto_adjust_amplifier_gain=%d") % value).c_str(),
        function_name);
      return value > 0;
    } else {
      LogMessage("CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return false;
}

std::string DmfControlBoard::waveform() {
  const char* function_name = "waveform()";
  LogSeparator();
  LogMessage("send command", function_name);
  if (SendCommand(CMD_GET_WAVEFORM) == RETURN_OK) {
    LogMessage("CMD_GET_WAVEFORM", function_name);
    if (payload_length() == 1) {
      uint8_t waveform = ReadUint8();
      std::string waveform_str;
      if (waveform == SINE || waveform == SQUARE) {
        if (waveform == SINE) {
          waveform_str = "SINE";
        } else if (waveform == SQUARE) {
          waveform_str = "SQUARE";
        }
        LogMessage(str(format("waveform=%s") % waveform_str).c_str(),
          function_name);
        return waveform_str;
      } else {
        return_code_ = RETURN_BAD_VALUE;
        LogMessage("CMD_GET_WAVEFORM, Bad value",
                   function_name);
      }
    } else {
      LogMessage("CMD_GET_WAVEFORM, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return "";
}

float DmfControlBoard::waveform_voltage() {
  const char* function_name = "waveform_voltage()";
  LogSeparator();
  LogMessage("send command", function_name);
  if (SendCommand(CMD_GET_WAVEFORM_VOLTAGE) == RETURN_OK) {
    LogMessage("CMD_GET_WAVEFORM_VOLTAGE", function_name);
    if (payload_length() == sizeof(float)) {
      float v_rms = ReadFloat();
      LogMessage(str(format("waveform_voltage=%.1f") % v_rms).c_str(),
        function_name);
      return v_rms;
    } else {
      LogMessage("CMD_GET_WAVEFORM_VOLTAGE, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

float DmfControlBoard::waveform_frequency() {
  const char* function_name = "waveform_frequency()";
  LogSeparator();
  LogMessage("send command", function_name);
  if (SendCommand(CMD_GET_WAVEFORM_FREQUENCY) == RETURN_OK) {
    LogMessage("CMD_GET_WAVEFORM_FREQUENCY", function_name);
    if (payload_length() == sizeof(float)) {
      float freq_hz = ReadFloat();
      LogMessage(str(format("waveform_frequency=%.1f") % freq_hz).c_str(),
        function_name);
      return freq_hz;
    } else {
      LogMessage("CMD_GET_WAVEFORM_FREQUENCY, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

uint8_t DmfControlBoard::set_series_resistor_index(const uint8_t channel,
                                                   const uint8_t index) {
  const char* function_name = "set_series_resistor_index()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel, sizeof(channel));
  Serialize(&index, sizeof(index));
  if (SendCommand(CMD_SET_SERIES_RESISTOR_INDEX) == RETURN_OK) {
    LogMessage("CMD_SET_SERIES_RESISTOR_INDEX", function_name);
    LogMessage("series resistor index set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_series_resistance(const uint8_t channel,
                                               float resistance) {
  const char* function_name = "set_series_resistance()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel, sizeof(channel));
  Serialize(&resistance, sizeof(resistance));
  if (SendCommand(CMD_SET_SERIES_RESISTANCE) == RETURN_OK) {
    LogMessage("CMD_SET_SERIES_RESISTANCE", function_name);
    LogMessage("series resistance set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_series_capacitance(const uint8_t channel,
                                                float capacitance) {
  const char* function_name = "set_series_capacitance()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel, sizeof(channel));
  Serialize(&capacitance, sizeof(capacitance));
  if (SendCommand(CMD_SET_SERIES_CAPACITANCE) == RETURN_OK) {
    LogMessage("CMD_SET_SERIES_CAPACITANCE", function_name);
    LogMessage("series capacitance set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_amplifier_gain(float gain) {
  const char* function_name = "set_amplifier_gain()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&gain, sizeof(gain));
  if (SendCommand(CMD_SET_AMPLIFIER_GAIN) == RETURN_OK) {
    LogMessage("CMD_SET_AMPLIFIER_GAIN", function_name);
    LogMessage("amplifier gain set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_auto_adjust_amplifier_gain(bool on) {
  const char* function_name = "set_auto_adjust_amplifier_gain()";
  LogSeparator();
  LogMessage("send command", function_name);
  uint8_t value = on;
  Serialize(&value, sizeof(value));
  if (SendCommand(CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN) == RETURN_OK) {
    LogMessage("CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN", function_name);
    LogMessage("auto adjust amplifier gain set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_state_of_all_channels(const vector <uint8_t>
                                                   state) {
  const char* function_name = "set_state_of_all_channels()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&state[0],state.size() * sizeof(uint8_t));
  if (SendCommand(CMD_SET_STATE_OF_ALL_CHANNELS) == RETURN_OK) {
    LogMessage("CMD_SET_STATE_OF_ALL_CHANNELS", function_name);
    LogMessage("all channels set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_state_of_channel(const uint16_t channel,
                                              const uint8_t state) {
  const char* function_name = "set_state_of_channel()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel, sizeof(channel));
  Serialize(&state, sizeof(state));
  if (SendCommand(CMD_SET_STATE_OF_CHANNEL) == RETURN_OK) {
    LogMessage("CMD_SET_STATE_OF_CHANNEL", function_name);
    LogMessage("channel set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_waveform(bool waveform) {
  const char* function_name = "set_waveform()";
  LogSeparator();
  LogMessage("send command", function_name);
  uint8_t data = waveform;
  LogMessage(str(format("waveform=%d") % waveform).c_str(), function_name);
  Serialize(&data, sizeof(data));
  if (SendCommand(CMD_SET_WAVEFORM) == RETURN_OK) {
    LogMessage("CMD_SET_WAVEFORM", function_name);
    LogMessage("waveform set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_waveform_voltage(const float v_rms){
  const char* function_name = "set_waveform_voltage()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&v_rms, sizeof(v_rms));
  LogMessage(str(format("waveform_voltage=%.1f") % v_rms).c_str(),
    function_name);
  if (SendCommand(CMD_SET_WAVEFORM_VOLTAGE) == RETURN_OK) {
    LogMessage("CMD_SET_WAVEFORM_VOLTAGE", function_name);
    LogMessage("voltage set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_waveform_frequency(const float freq_hz) {
  const char* function_name = "set_waveform_frequency()";
  LogSeparator();
  LogMessage(str(format("freq_hz=%.1f") % freq_hz).c_str(), function_name);
  LogMessage("send command", function_name);
  Serialize(&freq_hz, sizeof(freq_hz));
  if (SendCommand(CMD_SET_WAVEFORM_FREQUENCY) == RETURN_OK) {
    LogMessage("CMD_SET_WAVEFORM_FREQUENCY", function_name);
    LogMessage("frequency set successfully", function_name);
  }
  return return_code();
}

std::vector <float> DmfControlBoard::MeasureImpedance(
        uint16_t sampling_time_ms, uint16_t n_samples,
        uint16_t delay_between_samples_ms, const std::vector <uint8_t> state) {
    MeasureImpedanceNonBlocking(sampling_time_ms,
                              n_samples,
                              delay_between_samples_ms,
                              state);
  boost::this_thread::sleep(boost::posix_time::milliseconds(
      n_samples * (delay_between_samples_ms + sampling_time_ms)));
  return GetImpedanceData();
}

void DmfControlBoard::MeasureImpedanceNonBlocking(
        uint16_t sampling_time_ms, uint16_t n_samples,
        uint16_t delay_between_samples_ms, const std::vector <uint8_t> state) {
  const char* function_name = "MeasureImpedanceNonBlocking()";
  LogSeparator();
  LogMessage("send command", function_name);
  // if we get this far, everything is ok
  Serialize(&sampling_time_ms, sizeof(sampling_time_ms));
  Serialize(&n_samples, sizeof(n_samples));
  Serialize(&delay_between_samples_ms, sizeof(delay_between_samples_ms));
  Serialize(&state[0],state.size() * sizeof(uint8_t));
  SendNonBlockingCommand(CMD_MEASURE_IMPEDANCE);
}

std::vector < float> DmfControlBoard::GetImpedanceData() {
  const char* function_name = "GetImpedanceData()";
  if (ValidateReply(CMD_MEASURE_IMPEDANCE) == RETURN_OK) {
    uint16_t n_samples = payload_length() / (2 * sizeof(int16_t) + 2 *
                                             sizeof(int8_t));
    LogMessage(str(format("Read %d impedance samples") % n_samples).c_str(),
      function_name);
    std::vector < float> impedance_buffer(4 * n_samples);
    for (uint16_t i = 0; i < n_samples; i++) {
      impedance_buffer[4 * i] = (float)ReadInt16() * 5.0 / 1023 /  // V_hv
                                sqrt(2) / 2;
      impedance_buffer[4 * i + 1] = ReadInt8(); // hv_resistor
      impedance_buffer[4 * i + 2] = (float)ReadInt16() * 5.0 / 1023 /  // V_fb
                                    sqrt(2) / 2;
      impedance_buffer[4 * i + 3] = ReadInt8(); // fb_resistor
    }
    LogMessage(str(format("payload_length()=%d") % payload_length()).c_str(),
               function_name);
    LogMessage(str(format("bytes_read() - payload_length()=%d")
               % (bytes_read() - payload_length())).c_str(), function_name);
    return impedance_buffer;
  }
  return std::vector < float > (); // return an empty vector
}

uint8_t DmfControlBoard::ResetConfigToDefaults() {
  const char* function_name = "ResetConfigToDefaults()";
  LogSeparator();
  if (SendCommand(CMD_RESET_CONFIG_TO_DEFAULTS) == RETURN_OK) {
    LogMessage("CMD_RESET_CONFIG_TO_DEFAULTS", function_name);
    LogMessage("config reset successfully", function_name);
  }
  return return_code();
}

#endif // AVR
