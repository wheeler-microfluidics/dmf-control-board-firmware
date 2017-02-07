#if defined(AVR) || defined(__SAM3X8E__)

#include "FeedbackController.h"
#include "AdvancedADC.h"
#include "DMFControlBoard.h"

uint16_t FeedbackController::n_samples_per_window_;
DMFControlBoard* FeedbackController::parent_;
bool FeedbackController::rms_;
bool FeedbackController::current_limit_exceeded_;
FeedbackController::ADCChannel FeedbackController::channels_[] = {
    FeedbackController::ADCChannel(),
    FeedbackController::ADCChannel()
};

void FeedbackController::begin(DMFControlBoard* parent,
                               uint8_t hv_channel,
                               uint8_t fb_channel) {
  parent_ = parent;
  channels_[HV_CHANNEL_INDEX].channel = hv_channel;
  channels_[FB_CHANNEL_INDEX].channel = fb_channel;

  // set the default sampling rate
  AdvancedADC.setSamplingRate(35e3);

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

  // turn on smallest series resistors
  set_series_resistor_index(0, series_resistor_index(0));
  set_series_resistor_index(1, series_resistor_index(1));

  // initialize virtual ground levels
  for (uint8_t channel_index = 0; channel_index < NUMBER_OF_ADC_CHANNELS;
       channel_index++) {
    // get a reference to the channel
    ADCChannel& channel = channels_[channel_index];

    // integer
    channel.vgnd = 512;

    // convert to a float
    channel.vgnd_exp_filtered = channel.vgnd;

    // mark prev_saturated as true for initial sampling window
    channel.prev_saturated = true;
  }
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
  } else { // bad channel_index
    return_code = DMFControlBoard::RETURN_BAD_INDEX;
  }
  if (return_code==DMFControlBoard::RETURN_OK) {
    channels_[channel_index].series_resistor_index = index;
  }
  return return_code;
}

// update buffer with new reading and increment to next channel_index
void FeedbackController::interleaved_callback(uint8_t channel_index,
                                              uint16_t value) {
  // get a reference to the channel
  ADCChannel& channel = channels_[channel_index];

  // check if we've exceeded the saturation threshold
  if (value > SATURATION_THRESHOLD_HIGH || value < SATURATION_THRESHOLD_LOW) {
    if (channel.post_saturation_ignore == 0) {
      channel.saturated = channel.saturated + 1;
      // The ADC is saturated, so use a smaller resistor
      if (channel.series_resistor_index > 0) {
        set_series_resistor_index(channel_index,
            channel.series_resistor_index - 1);
      } else {
        // if we're saturating the smallest resistor, we've exceeded the
        // maximum current limit.
        current_limit_exceeded_ = true;
      }
      channel.post_saturation_ignore = N_IGNORE_POST_SATURATION;
    } else {
      channel.post_saturation_ignore -= 1;
    }
  } else {
    if (rms_) {
      // update rms
      int32_t v = (int32_t)((int16_t)value - (int16_t)channel.vgnd);
      channel.sum2 += v * v;
      channel.sum += value;
    } else {
      // update peak-to-peak
      if (value > channel.max_value) {
        channel.max_value = value;
      } else if (value < channel.min_value) {
        channel.min_value = value;
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

uint8_t FeedbackController::measure_impedance(float sampling_window_ms,
                                          uint16_t n_sampling_windows,
                                          float delay_between_windows_ms,
                                          float frequency,
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

  // save the rms flag
  rms_ =  rms;

  // find a sampling rate and n_samples_per_window combination that minimizes
  // the rms bias
  float sampling_rate;
  if (interleave_samples) {
    find_sampling_rate(sampling_window_ms, frequency,
                       MAX_SAMPLING_RATE/2, &sampling_rate,
                       &n_samples_per_window_);
    sampling_rate *= 2;
  } else {
    find_sampling_rate(sampling_window_ms/2, frequency,
                       MAX_SAMPLING_RATE, &sampling_rate,
                       &n_samples_per_window_);
  }

  // n_samples_per_window includes both channels
  n_samples_per_window_ *= 2;

  AdvancedADC.setSamplingRate(sampling_rate);

  int8_t resistor_index[NUMBER_OF_ADC_CHANNELS];
  uint8_t original_resistor_index[NUMBER_OF_ADC_CHANNELS];

  for (uint8_t channel_index = 0; channel_index < NUMBER_OF_ADC_CHANNELS;
       channel_index++) {
    // record the current series resistors (so we can restore the current state
    // after this measurement)
    original_resistor_index[channel_index] = \
      channels_[channel_index].series_resistor_index;

    // enable the largest series resistors
    resistor_index[channel_index] = \
      parent_->config_settings().n_series_resistors(channel_index) - 1;
    set_series_resistor_index(channel_index, resistor_index[channel_index]);
  }

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
  uint8_t channels[] = { channels_[HV_CHANNEL_INDEX].channel,
                         channels_[FB_CHANNEL_INDEX].channel
  };
  if (interleave_samples) {
    AdvancedADC.setBufferLen(n_samples_per_window_);
    AdvancedADC.setChannels((uint8_t*)channels, NUMBER_OF_ADC_CHANNELS);
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
        series_resistance(channels_[HV_CHANNEL_INDEX].channel, i);
    float C = parent_->config_settings(). \
        series_capacitance(channels_[HV_CHANNEL_INDEX].channel, i);
    #if ___HARDWARE_MAJOR_VERSION___ == 1
      transfer_function[i] = a_conv * sqrt(pow(10e6 / R + 1, 2) +
          pow(10e6 * C * 2 * M_PI * parent_->waveform_frequency(), 2));
    #else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
      transfer_function[i] = a_conv * sqrt(pow(10e6 / R, 2) +
          pow(10e6 * C * 2 * M_PI * parent_->waveform_frequency(), 2));
    #endif // #if ___HARDWARE_MAJOR_VERSION___ == 1
  }

  current_limit_exceeded_ = false;

  for (uint16_t i = 0; i < n_sampling_windows; i++) {
    for (uint8_t channel_index = 0; channel_index < NUMBER_OF_ADC_CHANNELS;
         channel_index++) {
      // get a reference to the channel
      ADCChannel& channel = channels_[channel_index];

      // set channel to the unsaturated state
      channel.saturated = 0;
      channel.post_saturation_ignore = 0;

      if (rms_) {
        // initialize sum and sum2 buffers
        channel.sum = 0;
        channel.sum2 = 0;
      } else {
        // initialize max/min buffers
        channel.min_value = 1023;
        channel.max_value = 0;
      }
    }

    // Sample for `sampling_window_ms` milliseconds
    if (interleave_samples) {
      AdvancedADC.begin();
      while(!AdvancedADC.finished());
    } else {
      AdvancedADC.setChannel(channels_[HV_CHANNEL_INDEX].channel);
      AdvancedADC.registerCallback(&hv_channel_callback);
      AdvancedADC.begin();
      while(!AdvancedADC.finished());
      AdvancedADC.setChannel(channels_[FB_CHANNEL_INDEX].channel);
      AdvancedADC.registerCallback(&fb_channel_callback);
      AdvancedADC.begin();
      while(!AdvancedADC.finished());
    }

    // start measuring the time after our sampling window completes
    long delta_t_start = micros();

    float measured_rms[NUMBER_OF_ADC_CHANNELS];
    uint16_t measured_pk_pk[NUMBER_OF_ADC_CHANNELS];

    for (uint8_t channel_index = 0; channel_index < NUMBER_OF_ADC_CHANNELS;
         channel_index++) {

      if (current_limit_exceeded_ && ___HARDWARE_MAJOR_VERSION___ > 1) {
        // If we've exceeded the current limit, it probably means that there's
        // a short, so turn off all channels and set the waveform voltage to 0.
        //
        // Note: Only throw current limit exception for hardware versions > 1.
        // Device impedance measurements are not actively used on this older
        // hardware and the current limit imposes restrictions on the number of
        // reservoir electrodes that can be actuated simultaneously (breaking
        // many legacy protocols).
        parent_->clear_all_channels();
        parent_->set_waveform_voltage(0, false);
        return parent_->RETURN_MAX_CURRENT_EXCEEDED;
      }

      // get a reference to the channel
      ADCChannel& channel = channels_[channel_index];

      // calculate the rms voltage
      if (rms_) {
        measured_rms[channel_index] = sqrt((float)channel.sum2 * \
            NUMBER_OF_ADC_CHANNELS / (float)(n_samples_per_window_));
        measured_pk_pk[channel_index] = (measured_rms[channel_index] * 64.0 * \
          2.0 * sqrt(2));

        if (channel.saturated == 0 && channel.prev_saturated == 0) {
          // Update the virtual ground for each channel. Note that we are
          // are calculating the average over the past 2 sampling windows (to
          // account for the possibility of having only half a waveform period),
          // then we are applying exponential smoothing.
          float alpha = 0.01; // Choosing a value for alpha is a trade-off
                              // between response time and precision.
                              // Increasing alpha allows vgnd to respond
                              // quickly, but since we don't expect it to
                              // change rapidly, we can keep alpha small to
                              // effectively average over more samples.
          channel.vgnd_exp_filtered = \
            alpha*(float)(channel.sum + channel.prev_sum) / \
            (float)(n_samples_per_window_) + \
            (1 - alpha)*channel.vgnd_exp_filtered;

          // store the actual vgnd value used in the callback function as an
          // integer for performance
          channel.vgnd = round(channel.vgnd_exp_filtered);

          // update prev_sum
          channel.prev_sum = channel.sum;
        }
      } else { // use peak-to-peak measurements
        measured_pk_pk[channel_index] = 64 * (channel.max_value - \
          channel.min_value);
        measured_rms[channel_index] = \
          measured_pk_pk[channel_index] / sqrt(2) / (2 * 64);
      }
      // update the resistor indices
      if (channel.saturated > 0) {
        resistor_index[channel_index] = -channel.saturated;
        channel.prev_saturated = true;
      } else {
        resistor_index[channel_index] = channel.series_resistor_index;
        channel.prev_saturated = false;
      }
    }

    // Based on the most-recent measurement of the incoming high-voltage
    // signal, adjust the gain correction factor we apply to waveform
    // amplitude changes to compensate for deviations from our model of the
    // gain of the amplifier. Only adjust gain if the last reading did not
    // saturate the ADC.
    if (parent_->auto_adjust_amplifier_gain() &&
        channels_[HV_CHANNEL_INDEX].saturated == 0 &&
        parent_->waveform_voltage() > 0) {

      // calculate the actuation voltage
      float measured_voltage = measured_rms[HV_CHANNEL_INDEX] * \
        transfer_function[resistor_index[HV_CHANNEL_INDEX]];

      float target_voltage;
      #if ___HARDWARE_MAJOR_VERSION___ == 1
        float V_fb;
        if (channels_[FB_CHANNEL_INDEX].saturated > 0) {
          V_fb = 0;
        } else {
          V_fb = measured_rms[FB_CHANNEL_INDEX] * a_conv;
        }
        target_voltage = parent_->waveform_voltage() + V_fb;
      #else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
        target_voltage = parent_->waveform_voltage();
      #endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1 / #else

      // The voltage tolerance represents the maximum error we are willing to
      // accept before the software throws an error message to the user.
      // We also want to define a secondary (lower) tolerance which will
      // trigger updating of the voltage/amplifier gain. If we update the
      // gain/voltage every sampling window, it can lead to oscillations,
      // so a simple solution is to define an acceptable window within which we
      // will ignore any difference between the target and set voltage (e.g.,
      // 50% of the voltage tolerance).
      //
      // Alternatively, we could use an adaptive updating scheme (e.g., PID
      // control or an exponential filter).
      if (abs(measured_voltage - target_voltage) >
          0.5 * parent_->config_settings().voltage_tolerance) {

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

    for (uint8_t channel_index = 0; channel_index < NUMBER_OF_ADC_CHANNELS;
         channel_index++) {
      // if the voltage on either channel is too low (< ~5% of the available
      // range), increase the series resistor index one step
      if (measured_pk_pk[channel_index] < (64*1023L)/20) {
        set_series_resistor_index(channel_index,
            channels_[channel_index].series_resistor_index + 1);
      }

      // Serialize measurements to the return buffer.
      parent_->serialize(&measured_pk_pk[channel_index], sizeof(uint16_t));
      parent_->serialize(&resistor_index[channel_index], sizeof(uint8_t));
    }

    // There is a new request available on the serial port.  Stop what we're
    // doing so we can service the new request.
    if (Serial.available() > 0) { break; }

    // if there is a delay between windows, wait until it has elapsed
    while( (micros() - delta_t_start) < delay_between_windows_ms*1000) {}
  }

  // Set the resistors back to their original states.
  for (uint8_t channel_index = 0; channel_index < NUMBER_OF_ADC_CHANNELS;
       channel_index++) {
    set_series_resistor_index(channel_index,
                              original_resistor_index[channel_index]);

  }

  return parent_->RETURN_OK;
}

void FeedbackController::find_sampling_rate(float sampling_window_ms,
                                          float frequency,
                                          float max_sampling_rate,
                                          float* sampling_rate_out,
                                          uint16_t* n_samples_per_window_out) {
  // Find the max sampling rate that will give an unbiased estimate of the rms
  // for a given sampling window length.
  //
  // Conditions:
  //   1. sampling rate < max sampling rate
  //   2. sampling rate is not a sub-harmonic of the fundamental frequency
  //   3. sampling rate / fundamental frequency is not an integer less than 10
  //   4. sampling rate / fundamental frequency * number of waveform periods
  //      is a multiple of 0.5
  // The following conditions also apply if the sampling rate is less than
  // the Nyquist rate:
  //   5. sampling rate is not a sub-harmonic of the aliased frequency
  //   6. sampling rate / aliased frequency is not an integer

  // calculate the minimum number of waveform periods (must be a multiple of
  // 0.5)
  float n_periods = round(float(sampling_window_ms) * frequency / 1000.0 / \
                          0.5) * 0.5;

  while (true) {
    // iterate over the possible number of samples per window (need at least 2)
    for (*n_samples_per_window_out = ceil(n_periods / frequency * \
                                          max_sampling_rate);
         *n_samples_per_window_out > 1;
         (*n_samples_per_window_out)--) {

      // calculate the sampling rate
      *sampling_rate_out = *n_samples_per_window_out/n_periods*frequency;

      // check conditions 1 and 2 (exclude sampling rates that are within 10%
      // of a sub-harmonic)
      if (*sampling_rate_out > max_sampling_rate || \
          abs(2.0 - round(2.0 / (*sampling_rate_out / frequency)) * \
              (*sampling_rate_out / frequency)) < 0.1) {
        continue;
      }

      // if we are sampling below the Nyquist rate, find the aliased frequency
      float f_aliased = frequency;
      if (*sampling_rate_out < 2 * frequency) {
        uint16_t k = 0;
        while (true) {
          if (frequency / (*sampling_rate_out) - k <= 0.5) {
            break;
          } else {
            k += 1;
          }
        }
        f_aliased = abs(frequency - k * (*sampling_rate_out));

        // check condition 5 (exclude sampling rates that are within 10%
        // of a sub-harmonic)
        if (abs(2.0 - round(2.0 / (*sampling_rate_out / f_aliased)) * \
            (*sampling_rate_out / f_aliased)) < 0.1) {
          continue;
        }

        // check condition 6
        if (abs(*sampling_rate_out / frequency - \
            round(*sampling_rate_out / frequency)) < 0.0001) {
          continue;
        }
      }

      // check condition 3
      if (*sampling_rate_out / frequency < 10 && \
          abs(*sampling_rate_out / frequency - \
          round(*sampling_rate_out / frequency)) < 0.0001) {
        continue;
      }

      // check condition 4
      if (abs(*sampling_rate_out / frequency * n_periods - \
          round(*sampling_rate_out / frequency * n_periods)) < 0.0001) {
        // if we get here, all conditions have been satisfied
        return;
      }
    }
    n_periods += 0.5;
  }
}

#endif // defined(AVR) || defined(__SAM3X8E__)
