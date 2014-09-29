#if defined(AVR) || defined(__SAM3X8E__)

#include "FeedbackController.h"
#include "AdvancedADC.h"
#include "DMFControlBoard.h"


uint8_t FeedbackController::series_resistor_indices_[NUMBER_OF_ADC_CHANNELS];
uint8_t FeedbackController::saturated_[NUMBER_OF_ADC_CHANNELS];
uint8_t FeedbackController::post_saturation_ignore_[NUMBER_OF_ADC_CHANNELS];
uint16_t FeedbackController::max_value_[NUMBER_OF_ADC_CHANNELS];
uint16_t FeedbackController::min_value_[NUMBER_OF_ADC_CHANNELS];
uint32_t FeedbackController::sum2_[NUMBER_OF_ADC_CHANNELS];
uint16_t FeedbackController::n_samples_per_window_;
DMFControlBoard* FeedbackController::parent_;
bool FeedbackController::rms_;

const uint8_t FeedbackController::CHANNELS[] = {
    FeedbackController::HV_CHANNEL,
    FeedbackController::FB_CHANNEL
};

void FeedbackController::begin(DMFControlBoard* parent) {
  parent_ = parent;

  // set the default sampling rate
  AdvancedADC.setSamplingRate(45e3);

  // Versions > 1.2 use the built in 5V AREF (default)
  #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ < 3
    analogReference(EXTERNAL);
  #endif

  #if ___HARDWARE_MAJOR_VERSION___ == 1
    pinMode(HV_SERIES_RESISTOR_0_, OUTPUT);
    pinMode(FB_SERIES_RESISTOR_0_, OUTPUT);
    pinMode(FB_SERIES_RESISTOR_1_, OUTPUT);
    pinMode(FB_SERIES_RESISTOR_2_, OUTPUT);
  #else
    pinMode(HV_SERIES_RESISTOR_0_, OUTPUT);
    pinMode(HV_SERIES_RESISTOR_1_, OUTPUT);
    pinMode(FB_SERIES_RESISTOR_0_, OUTPUT);
    pinMode(FB_SERIES_RESISTOR_1_, OUTPUT);
    pinMode(FB_SERIES_RESISTOR_2_, OUTPUT);
    pinMode(FB_SERIES_RESISTOR_3_, OUTPUT);
  #endif

  AdvancedADC.setPrescaler(3);

  // turn on smalles series resistors
  set_series_resistor_index(0, 0);
  set_series_resistor_index(1, 0);
}

uint8_t FeedbackController::set_series_resistor_index(
                                            const uint8_t channel_index,
                                            const uint8_t index) {
  uint8_t return_code = DMFControlBoard::RETURN_OK;
  if (channel_index==0) {
    switch(index) {
      case 0:
        digitalWrite(HV_SERIES_RESISTOR_0_, HIGH);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(HV_SERIES_RESISTOR_1_, LOW);
        #endif
        break;
      case 1:
        digitalWrite(HV_SERIES_RESISTOR_0_, LOW);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(HV_SERIES_RESISTOR_1_, HIGH);
        #endif
        break;
#if ___HARDWARE_MAJOR_VERSION___ == 2
      case 2:
        digitalWrite(HV_SERIES_RESISTOR_0_, LOW);
        digitalWrite(HV_SERIES_RESISTOR_1_, LOW);
        break;
#endif
      default:
        return_code = DMFControlBoard::RETURN_BAD_INDEX;
        break;
    }
    if (return_code==DMFControlBoard::RETURN_OK) {
      series_resistor_indices_[channel_index] = index;
    }
  } else if (channel_index==1) {
    switch(index) {
      case 0:
        digitalWrite(FB_SERIES_RESISTOR_0_, HIGH);
        digitalWrite(FB_SERIES_RESISTOR_1_, LOW);
        digitalWrite(FB_SERIES_RESISTOR_2_, LOW);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(FB_SERIES_RESISTOR_3_, LOW);
        #endif
        break;
      case 1:
        digitalWrite(FB_SERIES_RESISTOR_0_, LOW);
        digitalWrite(FB_SERIES_RESISTOR_1_, HIGH);
        digitalWrite(FB_SERIES_RESISTOR_2_, LOW);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(FB_SERIES_RESISTOR_3_, LOW);
        #endif
        break;
      case 2:
        digitalWrite(FB_SERIES_RESISTOR_0_, LOW);
        digitalWrite(FB_SERIES_RESISTOR_1_, LOW);
        digitalWrite(FB_SERIES_RESISTOR_2_, HIGH);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(FB_SERIES_RESISTOR_3_, LOW);
        #endif
        break;
      case 3:
        digitalWrite(FB_SERIES_RESISTOR_0_, LOW);
        digitalWrite(FB_SERIES_RESISTOR_1_, LOW);
        digitalWrite(FB_SERIES_RESISTOR_2_, LOW);
        #if ___HARDWARE_MAJOR_VERSION___ == 2
          digitalWrite(FB_SERIES_RESISTOR_3_, HIGH);
        #endif
        break;
      #if ___HARDWARE_MAJOR_VERSION___ == 2
      case 4:
        digitalWrite(FB_SERIES_RESISTOR_0_, LOW);
        digitalWrite(FB_SERIES_RESISTOR_1_, LOW);
        digitalWrite(FB_SERIES_RESISTOR_2_, LOW);
        digitalWrite(FB_SERIES_RESISTOR_3_, LOW);
        break;
      #endif
      default:
        return_code = DMFControlBoard::RETURN_BAD_INDEX;
        break;
    }
    if (return_code==DMFControlBoard::RETURN_OK) {
      series_resistor_indices_[channel_index] = index;
    }
  } else { // bad channel_index
    return_code = DMFControlBoard::RETURN_BAD_INDEX;
  }
  return return_code;
}

// update buffer with new reading and increment to next channel_index
void FeedbackController::interleaved_callback(uint8_t channel_index,
                                              uint16_t value) {
  // check if we've exceeded the saturation threshold
  if (value > SATURATION_THRESHOLD_HIGH || value < SATURATION_THRESHOLD_LOW) {
    if (post_saturation_ignore_[channel_index] == 0) {
      saturated_[channel_index] = saturated_[channel_index] + 1;
      // The ADC is saturated, so use a smaller resistor
      if (series_resistor_indices_[channel_index] > 0) {
        set_series_resistor_index(channel_index,
            series_resistor_indices_[channel_index] - 1);
      }
      post_saturation_ignore_[channel_index] = N_IGNORE_POST_SATURATION;
    } else {
      post_saturation_ignore_[channel_index] -= 1;
    }
  } else {
    if (rms_) {
      // update rms
      int32_t v = (int32_t)value-511;
      sum2_[channel_index] += v*v;
    } else {
      // update peak-to-peak
      if (value > max_value_[channel_index]) {
        max_value_[channel_index] = value;
      } else if (value < min_value_[channel_index]) {
        min_value_[channel_index] = value;
      }
    }
  }
}

void FeedbackController::hv_channel_callback(uint8_t channel_index,
                                             uint16_t value) {
  interleaved_callback(HV_CHANNEL_INDEX, value);
}

void FeedbackController::fb_channel_callback(uint8_t channel_index,
                                             uint16_t value) {
  interleaved_callback(FB_CHANNEL_INDEX, value);
}

uint16_t FeedbackController::measure_impedance(uint16_t n_samples_per_window,
                                          uint16_t n_sampling_windows,
                                          float delay_between_windows_ms,
                                          bool interleave_samples,
                                          bool rms) {
  // # `measure_impedance` #
  //
  // ## Pins ##
  //
  //  - Analog pin `0` is connected to _high-voltage (HV)_ signal.
  //  - Analog pin `1` is connected to _feedback (FB)_ signal.
  //
  // ## Analog input measurement circuit ##
  //
  //
  //                     $R_fixed$
  //     $V_in$   |-------/\/\/\------------\--------\--------\--------\
  //                                        |        |        |        |
  //                                        /        /        /        /
  //                              $R_min$   \  $R_1$ \  $R_2$ \  $R_3$ \
  //                                        /        /        /        /
  //                                        \        \        \        \
  //                                        |        |        |        |
  //                                        o        o        o        o
  //                                       /        /        /        /
  //                                      /        /        /        /
  //                                        o        o        o        o
  //                                        |        |        |        |
  //                                        |        |        |        |
  //                                       ---      ---      ---      ---
  //                                        -        -        -        -
  //
  // Based on the amplitude of $V_in$, we need to select an appropriate
  // resistor to activate, such that we divide the input voltage to within
  // 0-5V, since this is the range that Arduino ADC can handle.
  //
  // Collect samples over a set of windows of a set duration (number of
  // samples * sampling rate), and an optional delay between sampling windows.
  //
  // Only collect enough sampling windows to fill the maximum payload length.
  //
  // Each sampling window contains:
  //
  //  - High-voltage amplitude.
  //  - High-voltage resistor index.
  //  - Feedback amplitude.
  //  - Feedback resistor index.

  // save the number of samples per window and rms flag
  n_samples_per_window_  = n_samples_per_window;
  rms_ =  rms;

  // record the current series resistors (so we can restore the current state
  // after this measurement)
  uint8_t original_hv_index = series_resistor_indices_[HV_CHANNEL_INDEX];
  uint8_t original_fb_index = series_resistor_indices_[FB_CHANNEL_INDEX];

  // enable the largest series resistors
  int8_t hv_resistor_index = \
      parent_->config_settings().n_series_resistors(HV_CHANNEL_INDEX) - 1;
  int8_t fb_resistor_index = \
      parent_->config_settings().n_series_resistors(FB_CHANNEL_INDEX) - 1;

  set_series_resistor_index(HV_CHANNEL_INDEX, hv_resistor_index);
  set_series_resistor_index(FB_CHANNEL_INDEX, fb_resistor_index);

  // Sample the following voltage signals:
  //
  //  - Incoming high-voltage signal from the amplifier.
  //  - Incoming feedback signal from the DMF device.
  //
  // For each signal, take `n` samples to find the corresponding peak-to-peak
  // voltage.  While sampling, automatically select the largest feedback
  // resistor that _does not saturate_ the input _analog-to-digital converter
  // (ADC)_.  This provides the highest resolution measurements.
  //
  // __NB__ In the case where the signal saturates the lowest resistor, mark
  // the measurement as saturated/invalid.

  if (interleave_samples) {
    AdvancedADC.setBufferLen(n_samples_per_window_);
    AdvancedADC.setChannels((uint8_t*)CHANNELS, NUMBER_OF_ADC_CHANNELS);
    AdvancedADC.registerCallback(&interleaved_callback);
  } else {
    AdvancedADC.setBufferLen(n_samples_per_window_/2);
  }

  // Calculate the transfer function for each high-voltage attenuator. Do this
  // once prior to starting measurements because these floating point
  // calculations are computationally intensive.
  float a_conv = parent_->aref() / 1023.0;
  float transfer_function[
    sizeof(parent_->config_settings().A0_series_resistance)/sizeof(float)];
  for (uint8_t i=0; i < sizeof(transfer_function)/sizeof(float); i++) {
    float R = parent_->config_settings(). \
        series_resistance(HV_CHANNEL, i);
    float C = parent_->config_settings(). \
        series_capacitance(HV_CHANNEL, i);
    #if ___HARDWARE_MAJOR_VERSION___ == 1
      transfer_function[i] = a_conv * sqrt(pow(10e6 / R + 1, 2) +
          pow(10e6 * C * 2 * M_PI * parent_->waveform_frequency(), 2));
    #else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
      transfer_function[i] = a_conv * sqrt(pow(10e6 / R, 2) +
          pow(10e6 * C * 2 * M_PI * parent_->waveform_frequency(), 2));
    #endif // #if ___HARDWARE_MAJOR_VERSION___ == 1
  }

  for (uint16_t i = 0; i < n_sampling_windows; i++) {
    // set both channels to the unsaturated state
    saturated_[HV_CHANNEL_INDEX] = 0;
    saturated_[FB_CHANNEL_INDEX] = 0;
    post_saturation_ignore_[HV_CHANNEL_INDEX] = 0;
    post_saturation_ignore_[FB_CHANNEL_INDEX] = 0;

    if (rms_) {
      // initialize sum2 buffer
      sum2_[HV_CHANNEL_INDEX] = 0;
      sum2_[FB_CHANNEL_INDEX] = 0;
    } else {
      // initialize max/min buffers
      min_value_[HV_CHANNEL_INDEX] = 1023;
      max_value_[HV_CHANNEL_INDEX] = 0;
      min_value_[FB_CHANNEL_INDEX] = 1023;
      max_value_[FB_CHANNEL_INDEX] = 0;
    }

    // Sample for `sampling_window_ms` milliseconds
    if (interleave_samples) {
      AdvancedADC.begin();
      while(!AdvancedADC.finished());
    } else {
      AdvancedADC.setChannel(HV_CHANNEL);
      AdvancedADC.registerCallback(&hv_channel_callback);
      AdvancedADC.begin();
      while(!AdvancedADC.finished());
      AdvancedADC.setChannel(FB_CHANNEL);
      AdvancedADC.registerCallback(&fb_channel_callback);
      AdvancedADC.begin();
      while(!AdvancedADC.finished());
    }

    // start measuring the time after our sampling window completeds
    long delta_t_start = micros();

    float hv_rms, fb_rms;
    uint16_t hv_pk_pk, fb_pk_pk;

    // calculate the rms voltage for each channel
    if (rms_) {
      hv_rms = sqrt((float)sum2_[HV_CHANNEL_INDEX] * 2.0 / \
        (float)(n_samples_per_window_));
      fb_rms = sqrt((float)sum2_[FB_CHANNEL_INDEX] * 2.0 / \
        (float)(n_samples_per_window_));
      hv_pk_pk = (hv_rms * 64.0 * 2.0 * sqrt(2));
      fb_pk_pk = (fb_rms * 64.0 * 2.0 * sqrt(2));
    } else { // use peak-to-peak measurements
      hv_pk_pk = 64 * (max_value_[HV_CHANNEL_INDEX] - \
        min_value_[HV_CHANNEL_INDEX]);
      fb_pk_pk = 64 * (max_value_[FB_CHANNEL_INDEX] - \
        min_value_[FB_CHANNEL_INDEX]);
      hv_rms = hv_pk_pk / sqrt(2) / (2 * 64);
      fb_rms = fb_pk_pk / sqrt(2) / (2 * 64);
    }

    // update the resistor indices
    if (saturated_[HV_CHANNEL_INDEX] > 0) {
      hv_resistor_index = -saturated_[HV_CHANNEL_INDEX];
    } else {
      hv_resistor_index = series_resistor_indices_[HV_CHANNEL_INDEX];
    }
    if (saturated_[FB_CHANNEL_INDEX] > 0) {
      fb_resistor_index = -saturated_[FB_CHANNEL_INDEX];
    } else {
      fb_resistor_index = series_resistor_indices_[FB_CHANNEL_INDEX];
    }

    // Based on the most-recent measurement of the incoming high-voltage
    // signal, adjust the gain correction factor we apply to waveform
    // amplitude changes to compensate for deviations from our model of the
    // gain of the amplifier. Only adjust gain if the last reading did not
    // saturate the ADC.
    if (parent_->auto_adjust_amplifier_gain() &&
        saturated_[HV_CHANNEL_INDEX] == 0 &&
        parent_->waveform_voltage() > 0) {

      // calculate the actuation voltage
      float measured_voltage = hv_rms * transfer_function[hv_resistor_index];

      float target_voltage;
      #if ___HARDWARE_MAJOR_VERSION___ == 1
        float V_fb;
        if (saturated_[FB_CHANNEL_INDEX] > 0) {
          V_fb = 0;
        } else {
          V_fb = fb_rms * a_conv;
        }
        target_voltage = parent_->waveform_voltage() + V_fb;
      #else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
        target_voltage = parent_->waveform_voltage();
      #endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else

      // If we're outside of the voltage tolerance, update the gain and
      // voltage.
      if (abs(measured_voltage - target_voltage) >
          parent_->config_settings().voltage_tolerance) {

        float gain = parent_->amplifier_gain() * measured_voltage / \
          target_voltage;

        // Enforce minimum gain of 1 because if gain goes to zero, it cannot
        // be adjusted further.
        if (gain < 1) {
          gain = 1;
        }

        parent_->set_amplifier_gain(gain);
      }
    }

    // if the voltage on either channel is too low (< ~5% of the available
    // range), increase the series resistor index one step
    if (hv_pk_pk < (64*1023L)/20) {
      set_series_resistor_index(HV_CHANNEL_INDEX,
          series_resistor_indices_[HV_CHANNEL_INDEX] + 1);
    }
    if (fb_pk_pk < (64*1023L)/20) {
      set_series_resistor_index(FB_CHANNEL_INDEX,
          series_resistor_indices_[FB_CHANNEL_INDEX] + 1);
    }

    // Serialize measurements to the return buffer.
    parent_->serialize(&hv_pk_pk, sizeof(hv_pk_pk));
    parent_->serialize(&hv_resistor_index, sizeof(hv_resistor_index));
    parent_->serialize(&fb_pk_pk, sizeof(fb_pk_pk));
    parent_->serialize(&fb_resistor_index, sizeof(fb_resistor_index));

    // There is a new request available on the serial port.  Stop what we're
    // doing so we can service the new request.
    if (Serial.available() > 0) { break; }

    // if there is a delay between windows, wait until it has elapsed
    while( (micros() - delta_t_start) < delay_between_windows_ms*1000) {}
  }

  // Set the resistors back to their original states.
  set_series_resistor_index(HV_CHANNEL_INDEX, original_hv_index);
  set_series_resistor_index(FB_CHANNEL_INDEX, original_fb_index);

  // Return the number of samples that we measured _(i.e, the number of values
  // available in the result buffers)_.
  return n_sampling_windows;
}

#endif // defined(AVR) || defined(__SAM3X8E__)
