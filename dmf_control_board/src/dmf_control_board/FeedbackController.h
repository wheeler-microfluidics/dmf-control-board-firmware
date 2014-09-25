#ifndef _FEEDBACK_CONTROLLER_H_
#define _FEEDBACK_CONTROLLER_H_

#if defined(AVR) || defined(__SAM3X8E__)

#include "Arduino.h"
#include "Config.h"

class DMFControlBoard;

class FeedbackController {
public:

#if ___HARDWARE_MAJOR_VERSION___ == 1
  static const uint8_t HV_SERIES_RESISTOR_0_ = 13;
  static const uint8_t FB_SERIES_RESISTOR_0_ = 12;
  static const uint8_t FB_SERIES_RESISTOR_1_ = 11;
  static const uint8_t FB_SERIES_RESISTOR_2_ = 10;
#elif ___HARDWARE_MAJOR_VERSION___ == 2 && ___HARDWARE_MINOR_VERSION___ == 0
  static const uint8_t HV_SERIES_RESISTOR_0_ = 8;
  static const uint8_t HV_SERIES_RESISTOR_1_ = 9;
  static const uint8_t FB_SERIES_RESISTOR_0_ = 10;
  static const uint8_t FB_SERIES_RESISTOR_1_ = 11;
  static const uint8_t FB_SERIES_RESISTOR_2_ = 12;
  static const uint8_t FB_SERIES_RESISTOR_3_ = 13;
#else // 2.1
  static const uint8_t HV_SERIES_RESISTOR_0_ = 4;
  static const uint8_t HV_SERIES_RESISTOR_1_ = 5;
  static const uint8_t FB_SERIES_RESISTOR_0_ = 6;
  static const uint8_t FB_SERIES_RESISTOR_1_ = 7;
  static const uint8_t FB_SERIES_RESISTOR_2_ = 8;
  static const uint8_t FB_SERIES_RESISTOR_3_ = 9;
#endif
  static const uint16_t SATURATION_THRESHOLD_HIGH = 972; // ~95% max
  static const uint16_t SATURATION_THRESHOLD_LOW = 51; // ~5% max
  static const uint8_t N_IGNORE_POST_SATURATION = 3;

  // TODO:
  //  Eventually, all of these variables should defined only on the arduino.
  //  The PC can interogate device using CMD_GET_NUMBER_OF_ADC_CHANNELS
  static const uint8_t NUMBER_OF_ADC_CHANNELS = 2;
  static const uint8_t HV_CHANNEL = 0;
  static const uint8_t FB_CHANNEL = 2;
  static const uint8_t HV_CHANNEL_INDEX = 0;
  static const uint8_t FB_CHANNEL_INDEX = 1;
  static const uint8_t CHANNELS[];

  static void begin(DMFControlBoard* parent);
  static uint8_t set_series_resistor_index(const uint8_t channel_index,
                                    const uint8_t index);
  static uint8_t series_resistor_index(uint8_t channel_index) {
    return series_resistor_indices_[channel_index];
  }

  static uint16_t measure_impedance(uint16_t n_samples_per_window,
                             uint16_t n_sampling_windows,
                             float delay_between_windows_ms,
                             uint8_t options);
  static void interleaved_callback(uint8_t channel, uint16_t value);
  static void hv_channel_callback(uint8_t channel, uint16_t value);
  static void fb_channel_callback(uint8_t channel, uint16_t value);
private:
  static uint8_t series_resistor_indices_[NUMBER_OF_ADC_CHANNELS];
  static uint8_t saturated_[NUMBER_OF_ADC_CHANNELS];
  static uint8_t post_saturation_ignore_[NUMBER_OF_ADC_CHANNELS];
  static uint16_t max_value_[NUMBER_OF_ADC_CHANNELS];
  static uint16_t min_value_[NUMBER_OF_ADC_CHANNELS];
  static uint32_t sum2_[NUMBER_OF_ADC_CHANNELS];
  static uint16_t n_samples_per_window_;
  static DMFControlBoard* parent_;
  static bool rms_;
};
#endif // defined(AVR) || defined(__SAM3X8E__)

#endif // _FEEDBACK_CONTROLLER_H_
