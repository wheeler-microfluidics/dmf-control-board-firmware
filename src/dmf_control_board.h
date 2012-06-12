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

#ifndef AVR
  #include <iostream>
  #include <fstream>
  #include <string>
  #include <vector>
#endif
#include "RemoteObject.h"

class DmfControlBoard : public RemoteObject {
public:
  static const uint32_t BAUD_RATE = 115200;
  static const uint8_t SINE = 0;
  static const uint8_t SQUARE = 1;

  /**\brief EEPROM address of config settings.*/
  static const uint16_t EEPROM_CONFIG_SETTINGS = 100;

  struct version_t {
    uint16_t major;
    uint16_t minor;
    uint16_t micro;
  };

  struct config_settings_t {
    version_t version;

    /**\brief This byte sets the maximum output voltage for the waveform
    generator. It should be trimmed so that the output waveform is 4Vp-p when
    POT_WAVEOUT_GAIN_2 is set to 255.*/
    uint8_t waveout_gain_1;

    /**\brief This byte sets the value of the analog reference (between 0 and
    ~5V).*/
    uint8_t aref;

    /**\brief This byte sets the value of the virtual ground reference (between
    0 and 5V).*/
    uint8_t vgnd;

    float A0_series_resistance[2];
    float A0_series_capacitance[2];
    float A1_series_resistance[4];
    float A1_series_capacitance[4];
  };

  // TODO:
  //  Eventually, all of these variables should defined only on the arduino.
  //  The PC can interogate device using CMD_GET_NUMBER_OF_ADC_CHANNELS,
  //  CMD_GET_MAX_SAMPLES
  static const uint8_t NUMBER_OF_ADC_CHANNELS = 2;
  static const uint16_t MAX_SAMPLES =
                          (RemoteObject::MAX_PAYLOAD_LENGTH-1)/sizeof(uint16_t);

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
  static const uint8_t CMD_GET_SAMPLING_RATE =              0xAB;
  static const uint8_t CMD_SET_SAMPLING_RATE =              0xAC;
  static const uint8_t CMD_GET_SERIES_RESISTOR_INDEX =      0xAD;
  static const uint8_t CMD_SET_SERIES_RESISTOR_INDEX =      0xAE;
  static const uint8_t CMD_GET_SERIES_RESISTANCE =          0xAF;
  static const uint8_t CMD_SET_SERIES_RESISTANCE =          0xB0;
  static const uint8_t CMD_GET_SERIES_CAPACITANCE =         0xB1;
  static const uint8_t CMD_SET_SERIES_CAPACITANCE =         0xB2;
  static const uint8_t CMD_GET_AMPLIFIER_GAIN =             0xB3;
  static const uint8_t CMD_SET_AMPLIFIER_GAIN =             0xB4;

  // Other commands
  static const uint8_t CMD_SYSTEM_RESET =                   0xF1; //TODO
  static const uint8_t CMD_DEBUG_MESSAGE =                  0xF2; //TODO
  static const uint8_t CMD_DEBUG_ON =                       0xF3; //TODO
  static const uint8_t CMD_SAMPLE_VOLTAGE =                 0xF4;
  static const uint8_t CMD_MEASURE_IMPEDANCE =              0xF5;
  static const uint8_t CMD_GET_PEAK_VOLTAGE =               0xF6;

  //////////////////////////////////////////////////////////////////////////////
  //
  // Return codes:
  //
  // Return codes are uint8_t.  Some generic return codes are defined by the
  // RemoteObject class.  You may also create custom return codes as long as
  // they are > 0x10.
  //
  //////////////////////////////////////////////////////////////////////////////

  DmfControlBoard();
  ~DmfControlBoard();

// In our case, the PC is the only one sending commands
#ifndef AVR
  uint16_t number_of_channels();
  std::vector<uint8_t> state_of_all_channels();
  uint8_t state_of_channel(const uint16_t channel);
  float sampling_rate();
  uint8_t series_resistor_index(const uint8_t channel);
  float series_resistance(const uint8_t channel);
  float series_capacitance(const uint8_t channel);
  std::string waveform();
  float waveform_frequency();
  float waveform_voltage();
  float amplifier_gain();

  // Remote mutators (return code is from reply packet)
  uint8_t set_state_of_channel(const uint16_t channel, const uint8_t state);
  uint8_t set_state_of_all_channels(const std::vector<uint8_t> state);
  uint8_t set_waveform_voltage(const float v_rms);
  uint8_t set_waveform_frequency(const float freq_hz);
  uint8_t set_waveform(bool waveform);
  uint8_t set_sampling_rate(const uint8_t sampling_rate);
  uint8_t set_series_resistor_index(const uint8_t channel,
                                    const uint8_t index);
  uint8_t set_series_resistance(const uint8_t channel,
                                float resistance);
  uint8_t set_series_capacitance(const uint8_t channel,
                                 float capacitance);
  uint8_t set_amplifier_gain(float gain);

  // other functions
  std::vector<float> SampleVoltage(
                          std::vector<uint8_t> ad_channel,
                          uint16_t n_samples,
                          uint16_t n_sets,
                          uint16_t delay_between_sets_ms,
                          const std::vector<uint8_t> state);
  std::vector<int16_t> MeasureImpedance(
                          uint16_t sampling_time_ms,
                          uint16_t n_samples,
                          uint16_t delay_between_samples_ms,
                          const std::vector<uint8_t> state);
  float GetPeakVoltage(uint8_t adc_channel,
		  	  	  	   uint16_t sampling_time_ms);
  uint8_t SetExperimentLogFile(const char* file_name);
  void LogExperiment(const char* message);
  std::string host_name() { return NAME_; }
  std::string host_manufacturer() { return MANUFACTURER_; }
  std::string host_hardware_version() { return HARDWARE_VERSION_; }
  std::string host_software_version() { return SOFTWARE_VERSION_; }
  std::string host_url() { return URL_; }
#else
  void begin();
  void PeakExceeded();

  // local accessors
  const char* protocol_name() { return PROTOCOL_NAME_; }
  const char* protocol_version() { return PROTOCOL_VERSION_; }
  const char* name() { return NAME_; } //device name
  const char* manufacturer() { return MANUFACTURER_; }
  const char* software_version() { return SOFTWARE_VERSION_; }
  const char* hardware_version() { return HARDWARE_VERSION_; }
  const char* url() { return URL_; }
#endif

private:
  // private static members
  static const char SOFTWARE_VERSION_[];
  static const char NAME_[];
  static const char HARDWARE_VERSION_[];
  static const char MANUFACTURER_[];
  static const char URL_[];
#ifdef AVR
  static const char PROTOCOL_NAME_[];
  static const char PROTOCOL_VERSION_[];

  static const uint8_t AD5204_SLAVE_SELECT_PIN_ = 53; // digital pot
  static const uint8_t POT_INDEX_AREF_ = 0;
  static const uint8_t POT_INDEX_VGND_ = 1;
  static const uint8_t POT_INDEX_WAVEOUT_GAIN_1_ = 2;
  static const uint8_t POT_INDEX_WAVEOUT_GAIN_2_ = 3;
  static const uint8_t HV_PEAK_ = 3;
  static const uint8_t FB_PEAK_ = 2;

  static const uint8_t PWR_SUPPLY_ON_ = 8;
  static const uint8_t WAVEFORM_SELECT_ = 9;

  static const uint8_t A0_SERIES_RESISTOR_0_ = 13;
  static const uint8_t A1_SERIES_RESISTOR_0_ = 12;
  static const uint8_t A1_SERIES_RESISTOR_1_ = 11;
  static const uint8_t A1_SERIES_RESISTOR_2_ = 10;

  static const uint8_t HV_PEAK_INTERRUPT_ = 1;
  static const uint8_t FB_PEAK_INTERRUPT_ = 0;

  static const float SAMPLING_RATES_[];

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
  static const uint8_t PCA9505_ADDRESS_ = 0x20;
  static const uint8_t PCA9505_CONFIG_IO_REGISTER_ = 0x18;
  static const uint8_t PCA9505_OUTPUT_PORT_REGISTER_ = 0x08;

  // LTC6904 (programmable oscillator) chip address
  static const uint8_t LTC6904_ = 0x17;
#else
  static const char CSV_INDENT_[];
#endif

  // private functions
  virtual uint8_t ProcessCommand(uint8_t cmd);
#ifdef AVR
  uint8_t UpdateChannel(const uint16_t channel, const uint8_t state);
  void UpdateAllChannels();
  void SendI2C(uint8_t row, uint8_t cmd, uint8_t data);
  void SendSPI(uint8_t pin, uint8_t address, uint8_t data);
  uint8_t SetPot(uint8_t index, uint8_t value);
  uint8_t SetSeriesResistor(const uint8_t channel,
                            const uint8_t index);
  uint8_t SetAdcPrescaler(const uint8_t index);
  float GetPeakVoltage(const uint8_t interrupt, const uint16_t sample_time_ms);
  void LoadConfig();
  void SaveConfig();
  version_t ConfigVersion();
#else
  uint8_t SendCommand(const uint8_t cmd);
  float MillisecondsSinceLastCheck();
  void SerializeChannelState(const std::vector<uint8_t> state,
                               std::ostringstream& msg);
#endif

  //private members
#ifdef AVR
  uint16_t number_of_channels_;
  uint8_t sampling_rate_index_;
  uint8_t A0_series_resistor_index_;
  uint8_t A1_series_resistor_index_;
  uint8_t peak_;
  float waveform_voltage_;
  float waveform_frequency_;
  float amplifier_gain_;
  config_settings_t config_settings_;
  #else
  std::string experiment_log_file_name_;
  std::ofstream experiment_log_file_;
  boost::posix_time::ptime t_last_check_;
#endif
};
#endif // _DMF_CONTROL_BOARD_H_

