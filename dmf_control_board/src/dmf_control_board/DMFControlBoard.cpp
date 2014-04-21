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
#endif

#ifdef AVR // only on Arduino Mega 2560
  const float DMFControlBoard::SAMPLING_RATES_[] = { 8908, 16611, 29253, 47458,
                                                     68191, 90293, 105263 };
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
                   ,"DMFControlBoard" //used for logging
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
#if ___HARDWARE_MAJOR_VERSION___ == 1
        return_code_ = RETURN_OK;
        serialize(&waveform_voltage_, sizeof(waveform_voltage_));
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
                     (uint8_t *)&data[0], 5);
            return_code_ = data[4];
            if (return_code_ == RETURN_OK) {
              memcpy(&waveform_voltage_, &data[0], sizeof(float));
              waveform_voltage_ *= amplifier_gain_;
              serialize(&waveform_voltage_, sizeof(float));
            }
          }
        }
#endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else
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
#if ___HARDWARE_MAJOR_VERSION___ == 1
        // the frequency of the LTC6904 oscillator needs to be set to 50x
        // the fundamental frequency
        waveform_frequency_ = read_float();
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
#else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
        waveform_frequency_ = read_float();
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
#endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#ifdef AVR // only on Arduino Mega 2560
    case CMD_GET_SAMPLING_RATE:
      if (payload_length() == 0) {
        serialize(&SAMPLING_RATES_[sampling_rate_index_], sizeof(float));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SAMPLING_RATE:
      if (payload_length() == sizeof(uint8_t)) {
        return_code_ = set_adc_prescaler(read_uint8());
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#endif
    case CMD_GET_SERIES_RESISTOR_INDEX:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t channel = read_uint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            serialize(&A0_series_resistor_index_,
                      sizeof(A0_series_resistor_index_));
            break;
          case 1:
            serialize(&A1_series_resistor_index_,
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
        uint8_t channel = read_uint8();
        return_code_ = set_series_resistor(channel, read_uint8());
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_SERIES_RESISTANCE:
      if (payload_length() == sizeof(uint8_t)) {
        uint8_t channel = read_uint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            serialize(&config_settings_.A0_series_resistance
                      [A0_series_resistor_index_], sizeof(float));
            break;
          case 1:
            serialize(&config_settings_.A1_series_resistance
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
        uint8_t channel = read_uint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            config_settings_.A0_series_resistance[A0_series_resistor_index_] =
                read_float();
            save_config();
            break;
          case 1:
            config_settings_.A1_series_resistance[A1_series_resistor_index_] =
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
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            serialize(&config_settings_.A0_series_capacitance
                      [A0_series_resistor_index_], sizeof(float));
            break;
          case 1:
            serialize(&config_settings_.A1_series_capacitance
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
        uint8_t channel = read_uint8();
        return_code_ = RETURN_OK;
        switch(channel) {
          case 0:
            config_settings_.A0_series_capacitance[A0_series_resistor_index_] =
                read_float();
            save_config();
            break;
          case 1:
            config_settings_.A1_series_capacitance[A1_series_resistor_index_] =
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
      if (payload_length() < 3*sizeof(uint16_t)) {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      } else {
        uint16_t sampling_time_ms = read_uint16();
        uint16_t n_samples = read_uint16();
        uint16_t delay_between_samples_ms = read_uint16();

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
          set_series_resistor(0, sizeof(config_settings_.A0_series_resistance) /
                            sizeof(float) - 1);
          set_series_resistor(1, sizeof(config_settings_.A1_series_resistance) /
                            sizeof(float) - 1);

          // update the channels (if they were included in the packet)
          if (payload_length() == 3 * sizeof(uint16_t) + number_of_channels_ *
              sizeof(uint8_t)) {
            update_all_channels();
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

            while (millis() - t_sample < sampling_time_ms) {
              // hv_resistor == -1 if the smallest series resistor becomes
              // saturated
              if (hv_resistor != -1) {
                hv = analogRead(0);
                // if the ADC is saturated, use a smaller resistor
                // and reset the peak
                if (hv > 820) {
                  if (A0_series_resistor_index_ > 0) {
                    set_series_resistor(0, A0_series_resistor_index_ - 1);
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
                    set_series_resistor(1, A1_series_resistor_index_ - 1);
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
            if (fb_max == 0 || (fb_min == 1023 && fb_resistor != -1)) {
              fb_resistor = -1;
            } else {
              fb_resistor = A1_series_resistor_index_;
            }

            // if we didn't get a valid high voltage sample during the
            // sampling time, return -1 as the index
            if (hv_max == 0 || (hv_min == 1023 && hv_resistor != -1)) {
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
#else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
                measured_voltage = (hv_pk_pk * 5.0 / 1023.0  // measured Vrms /
                                    / sqrt(2) / 2) /
                                   (1 / sqrt(pow(10e6 / R,   // transfer
                                                 2) +        // function
                                             pow(10e6 * C * 2 * M_PI *
                                                 waveform_frequency_, 2)));
                set_voltage = waveform_voltage_;
#endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else
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
                  set_waveform_voltage(waveform_voltage_ + V_fb);
#else   // #if ___HARDWARE_MAJOR_VERSION___ == 1
                  // update output voltage (but don't wait for i2c response)
                  set_waveform_voltage(waveform_voltage_, false);
#endif // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else
                }
              }
              hv_resistor = A0_series_resistor_index_;
            }

            serialize(&hv_pk_pk, sizeof(hv_pk_pk));
            serialize(&hv_resistor, sizeof(hv_resistor));
            serialize(&fb_pk_pk, sizeof(fb_pk_pk));
            serialize(&fb_resistor, sizeof(fb_resistor));

            if (Serial.available() > 0) {
              break;
            }

            uint32_t t_delay = millis();
            while (millis() - t_delay < delay_between_samples_ms) {}
          }

          // set the resistors back to their original states
          set_series_resistor(0, original_A0_index);
          set_series_resistor(1, original_A1_index);
        } else {
          return_code_ = RETURN_BAD_PACKET_SIZE;
        }
      }
      break;
    case CMD_RESET_CONFIG_TO_DEFAULTS:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        load_config(true);
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#if (___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ > 1) ||\
        ___HARDWARE_MAJOR_VERSION___ == 2
    case CMD_GET_POWER_SUPPLY_PIN:
      if (payload_length() == 0) {
        return_code_ = RETURN_OK;
        uint8_t power_supply_pin = POWER_SUPPLY_ON_PIN_;
        serialize(&power_supply_pin, sizeof(power_supply_pin));
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#endif
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
          watchdog_enabled(true);
        } else {
          watchdog_enabled(false);
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
          watchdog_state(true);
        } else {
          watchdog_state(false);
        }
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#ifdef ATX_POWER_SUPPLY
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
#endif  // ATX_POWER_SUPPLY
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
    pinMode(POWER_SUPPLY_ON_PIN_, OUTPUT);
    digitalWrite(POWER_SUPPLY_ON_PIN_, LOW);

    // wait for the power supply to turn on
    delay(500);
  #endif

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

  load_config();
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
  #else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
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
  #endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else

  Serial.print("switching_board_i2c_address=");
  Serial.println(config_settings_.switching_board_i2c_address, DEC);
  Serial.print("amplifier_gain=");
  Serial.println(config_settings_.amplifier_gain);
  Serial.print("voltage_tolerance=");
  Serial.println(config_settings_.voltage_tolerance);

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

  set_series_resistor(0, 0);
  set_series_resistor(1, 0);
#ifdef AVR // only on Arduino Mega 2560
  set_adc_prescaler(4);
#endif
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

#ifdef AVR // only on Arduino Mega 2560
uint8_t DMFControlBoard::set_adc_prescaler(const uint8_t index) {
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
#endif
uint8_t DMFControlBoard::set_series_resistor(const uint8_t channel,
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

uint8_t DMFControlBoard::set_waveform_voltage(const float output_vrms,
                                              const bool wait_for_reply) {
  uint8_t return_code;
#if ___HARDWARE_MAJOR_VERSION___==1
  float step = output_vrms / amplifier_gain_ * 2 * sqrt(2) / 4 * 255;
  if (output_vrms < 0 || step > 255) {
    return_code = RETURN_BAD_VALUE;
  } else {
    waveform_voltage_ = output_vrms;
    set_pot(POT_INDEX_WAVEOUT_GAIN_2_, step);
    return_code = RETURN_OK;
  }
#else  // #if ___HARDWARE_MAJOR_VERSION___==1
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
#endif  // #if ___HARDWARE_MAJOR_VERSION___==1 / #else
  return return_code;
}

void /* DEVICE */ DMFControlBoard::persistent_write(uint16_t address,
                                                    uint8_t value) {
    RemoteObject::persistent_write(address, value);
    /* Reload config from persistent storage _(e.g., EEPROM)_ to refresh
     * `ConfigSettings` struct with new value. */
    load_config();
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

float DMFControlBoard::sampling_rate() {
  return send_read_command<float>(CMD_GET_SAMPLING_RATE,
                                  "sampling_rate()");
}

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

uint8_t DMFControlBoard::set_sampling_rate(const uint8_t sampling_rate) {
    return send_set_command(CMD_SET_SAMPLING_RATE, "set_sampling_rate()",
                            sampling_rate);
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
                                          uint16_t sampling_time_ms,
                                          uint16_t n_samples,
                                          uint16_t delay_between_samples_ms,
                                          const std::vector <uint8_t> state) {
  measure_impedance_non_blocking(sampling_time_ms,
                                 n_samples,
                                 delay_between_samples_ms,
                                 state);
  boost::this_thread::sleep(boost::posix_time::milliseconds(
      n_samples * (delay_between_samples_ms + sampling_time_ms)));
  return get_impedance_data();
}

void DMFControlBoard::measure_impedance_non_blocking(
                                          uint16_t sampling_time_ms,
                                          uint16_t n_samples,
                                          uint16_t delay_between_samples_ms,
                                          const std::vector <uint8_t> state) {
  const char* function_name = "measure_impedance_non_blocking()";
  log_separator();
  log_message("send command", function_name);
  // if we get this far, everything is ok
  serialize(&sampling_time_ms, sizeof(sampling_time_ms));
  serialize(&n_samples, sizeof(n_samples));
  serialize(&delay_between_samples_ms, sizeof(delay_between_samples_ms));
  serialize(&state[0],state.size() * sizeof(uint8_t));
  send_non_blocking_command(CMD_MEASURE_IMPEDANCE);
}

std::vector<float> DMFControlBoard::get_impedance_data() {
  const char* function_name = "get_impedance_data()";
  if (validate_reply(CMD_MEASURE_IMPEDANCE) == RETURN_OK) {
    uint16_t n_samples = payload_length() / (2 * sizeof(int16_t) + 2 *
                                             sizeof(int8_t));
    log_message(str(format("Read %d impedance samples") % n_samples).c_str(),
                function_name);
    std::vector < float> impedance_buffer(4 * n_samples);
    for (uint16_t i = 0; i < n_samples; i++) {
      impedance_buffer[4 * i] = (float)read_int16() * 5.0 / 1023 /  // V_hv
                                sqrt(2) / 2;
      impedance_buffer[4 * i + 1] = read_int8(); // hv_resistor
      impedance_buffer[4 * i + 2] = (float)read_int16() * 5.0 / 1023 /  // V_fb
                                    sqrt(2) / 2;
      impedance_buffer[4 * i + 3] = read_int8(); // fb_resistor
    }
    log_message(str(format("payload_length()=%d") % payload_length()).c_str(),
                function_name);
    log_message(str(format("bytes_read() - payload_length()=%d")
                % (bytes_read() - payload_length())).c_str(), function_name);
    return impedance_buffer;
  }
  return std::vector<float>(); // return an empty vector
}

uint8_t DMFControlBoard::reset_config_to_defaults() {
  const char* function_name = "reset_config_to_defaults()";
  log_separator();
  if (send_command(CMD_RESET_CONFIG_TO_DEFAULTS) == RETURN_OK) {
    log_message("CMD_RESET_CONFIG_TO_DEFAULTS", function_name);
    log_message("config reset successfully", function_name);
  }
  return return_code();
}

#endif // defined(AVR) || defined(__SAM3X8E__)
