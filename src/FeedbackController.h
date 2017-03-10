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
  static const float MAX_SAMPLING_RATE = 40e3;

  struct ADCChannel {
    ADCChannel() : series_resistor_index(0) {}
    uint8_t channel;
    uint8_t series_resistor_index;
    uint8_t saturated;
    uint8_t post_saturation_ignore;
    uint16_t max_value;
    uint16_t min_value;
    uint32_t sum;
    uint32_t prev_sum;
    bool prev_saturated;
    uint32_t sum2;
    uint16_t vgnd;
    float vgnd_exp_filtered;
  };

  static const uint8_t NUMBER_OF_ADC_CHANNELS = 2;
  static const uint8_t HV_CHANNEL_INDEX = 0;
  static const uint8_t FB_CHANNEL_INDEX = 1;

  static void begin(DMFControlBoard* parent,
                    uint8_t hv_channel,
                    uint8_t fb_channel);
  static uint8_t set_series_resistor_index(const uint8_t channel_index,
                                           const uint8_t index);

  static uint8_t series_resistor_index(uint8_t channel_index) {
    return channels_[channel_index].series_resistor_index;
  }

  static uint8_t measure_impedance(float sampling_window_ms,
                             uint16_t n_sampling_windows,
                             float delay_between_windows_ms,
                             float frequency,
                             bool interleave_samples,
                             bool rms);
  static void interleaved_callback(uint8_t channel_index, uint16_t value);
  static void hv_channel_callback(uint8_t channel_index, uint16_t value);
  static void fb_channel_callback(uint8_t channel_index, uint16_t value);
  static ADCChannel* channels() { return channels_; }
private:
  static void find_sampling_rate(float sampling_window_ms,
                                 float frequency,
                                 float max_sampling_rate,
                                 float* sampling_rate_out,
                                 uint16_t* n_samples_per_window_out);
  static ADCChannel channels_[];
  static uint16_t n_samples_per_window_;
  static DMFControlBoard* parent_;
  static bool rms_;
  static bool current_limit_exceeded_;
};
#endif // defined(AVR) || defined(__SAM3X8E__)

#endif // _FEEDBACK_CONTROLLER_H_
