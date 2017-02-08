#include <Arduino.h>
#include "AdvancedADC.h"

#define TIMER1_RESOLUTION 65536UL  // Timer1 is 16 bit

uint8_t* ADCClass::buffer8_;
uint16_t* ADCClass::buffer16_;
volatile uint16_t ADCClass::current_index_;
uint16_t ADCClass::buffer_len_ = 0;
bool ADCClass::auto_trigger_on_;
uint8_t* ADCClass::channels_;
uint8_t ADCClass::channel_;
uint8_t ADCClass::n_channels_ = 0;
uint8_t ADCClass::channel_index_ = 0;
void (*ADCClass::update)();
void (*ADCClass::callback_)(uint8_t, uint16_t);
float ADCClass::sampling_rate_ = 0;
float ADCClass::measured_sampling_rate_ = 0;
unsigned long ADCClass::t_start_;
uint8_t ADCClass::clock_select_bits_;

ADCClass::ADCClass() {
  // use AVCC as a reference by default
  setAnalogReference(DEFAULT);
}

void ADCClass::setAnalogReference(uint8_t ref) {
  // ADMUX: REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
  ADMUX &= ~( (1 << 7) | (1 << 6) ); // clear the REFS1:0 bits
  ADMUX |= (ref << 6);
}

void ADCClass::setPrescaler(uint8_t prescaler) {
  // ADCSRA: ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
  ADCSRA &= ~0x07;  // clear the ADPS2:0 bits
  ADCSRA |= prescaler; // set the ADPS2:0 bits
}

uint16_t ADCClass::prescaler() {
  // ADCSRA: ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
  return ADCSRA & 0x07;
}

void ADCClass::setSamplingRate(float sampling_rate) {
  // TCCR1B: ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10
	TCCR1B = (1 << WGM13); // set mode as phase and frequency correct pwm, stop the timer

  // TCCR1A: COM1A1 COM1A0 COM1B1 COM1B0 COM1C1 COM1C0 WGM11 WGM10
	TCCR1A = 0;  // clear control register A

  uint16_t pwm_period;
	const unsigned long cycles = (F_CPU / 2000000) * (1e6/sampling_rate);
	if (cycles < TIMER1_RESOLUTION) {
		clock_select_bits_ = (1 << CS10);
		pwm_period = cycles;
	} else
	if (cycles < TIMER1_RESOLUTION * 8) {
		clock_select_bits_ = (1 << CS11);
		pwm_period = cycles / 8;
	} else
	if (cycles < TIMER1_RESOLUTION * 64) {
		clock_select_bits_ =(1 << CS11) | (1 << CS10);
		pwm_period = cycles / 64;
	} else
	if (cycles < TIMER1_RESOLUTION * 256) {
		clock_select_bits_ = (1 << CS12);
		pwm_period = cycles / 256;
	} else
	if (cycles < TIMER1_RESOLUTION * 1024) {
		clock_select_bits_ = (1 << CS12) | (1 << CS10);
		pwm_period = cycles / 1024;
	} else {
		clock_select_bits_ = (1 << CS12) | (1 << CS10);
		pwm_period = TIMER1_RESOLUTION - 1;
	}
	ICR1 = pwm_period;

  // TIMSK1: - - ICIE1 - OCIE1C OCIE1B OCIE1A TOIE1
	TIMSK1 = (1 << TOIE1); // enable the timer overflow interupt

  setTriggerSource(ADC_ATS_TIMER1_OVERFLOW);
  setAutoTrigger(true);

  sampling_rate_ = sampling_rate;
}

void ADCClass::_startTimer() {
  // TCCR1B: ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10
	_stopTimer();
	TCNT1 = 0;		// reset the counter
	TCCR1B |= clock_select_bits_; // start the timer
}

void ADCClass::_stopTimer() {
  // TCCR1B: ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10
	TCCR1B &= ~0x07; // clear the clock select bits
}

// option to left align the ADC values so we can read highest 8 bits from ADCH register only
void ADCClass::setLeftAlignResult(bool on) {
  // ADMUX: REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
  if (on) {
    ADMUX |= (1 << ADLAR); // turn on left alignment
  } else {
    ADMUX &= ~(1 << ADLAR); // turn off left alignment
  }
}

void ADCClass::setTriggerSource(uint8_t trigger_source) {
  // ADCSRB:  -  ACME  -   -  MUX5 ADTS2 ADTS1 ADTS0
  ADCSRB &= ~0x07; // clear the ADTS2:0 bits
  ADCSRB |= trigger_source; // set the ADTS2:0 bits
}

uint8_t ADCClass::triggerSource() {
  // ADCSRB:  -  ACME  -   -  MUX5 ADTS2 ADTS1 ADTS0
  return ADCSRB & 0x07;
}

void ADCClass::setAutoTrigger(bool on) {
  auto_trigger_on_ = on;
  // ADCSRA: ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
  if (on) {
    ADCSRA |= (1 << ADATE); // enable auto trigger mode
  } else {
    ADCSRA &= ~(1 << ADATE); // clear auto trigger
  }
}

void ADCClass::next() {
  ADCSRA |= (1 << ADSC); // start next conversion
}

void ADCClass::begin() {
  current_index_ = 0;

  // re/enable auto trigger; it gets cleared on stop()
  setAutoTrigger(auto_trigger_on_);

  // enable ADC, enable interrupts when measurement completes,
  // and start first conversion.
  //
  // ADCSRA: ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
  //
  // ADATE bit enabbles auto trigger
  // ADIE bit enable interrupts when measurement complete
  // ADEN bit enables the ADC
  // ADSC bit starts a conversion
  t_start_ = micros();
  ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADIE);

  if (triggerSource() == ADC_ATS_TIMER1_OVERFLOW) {
    _startTimer();
  }
}

void ADCClass::stop() {
  unsigned long t_end = micros();

  // disable auto trigger (ADATE) and disable interrupts when
  // measurement completes (ADIE)
  //
  // ADCSRA: ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
  ADCSRA &= ~( (1 << ADATE) | (1 << ADIE) );
  if (triggerSource() == ADC_ATS_TIMER1_OVERFLOW) {
    _stopTimer();
  }
  measured_sampling_rate_ = (float)buffer_len_*1e6/(float)(t_end - t_start_);
}

// publicly available method
// This clears any existing channel array
void ADCClass::setChannel(uint8_t channel) {
  // clear the channel array
  n_channels_ = 1;
  channels_ = 0;
  channel_index_ = 0;
  update = &updateSingleChannel;
  // set the ADC registers for this channel
  _setChannel(channel);
}

// private method for setting up ADC registers to read a channel
void ADCClass::_setChannel(uint8_t channel) {
  channel_ = channel;
  // set ADMUX register
  // ADMUX: REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
  ADMUX &= 0xF0; // clear MUX4:0
  ADMUX |= (channel & 0x07); // set MUX2:0

  // update MUX5 bit on ADCSRB register
  // ADCSRB:  -  ACME  -   -  MUX5 ADTS2 ADTS1 ADTS0
  if (channel > 7) {
    ADCSRB |= (1 << MUX5);
  } else {
    ADCSRB &= ~(1 << MUX5);
  }
}

void ADCClass::enableDigitalInputs(bool enabled) {
  //---------------------------------------------------------------------
  // DIDR0 and DIDR2 settings
  //---------------------------------------------------------------------
  // When this bit is written logic one, the digital input buffer on the
  // corresponding ADC pin is disabled. The corresponding PIN Register
  // bit will always read as zero when this bit is set. When an analog
  // signal is applied to the AD7:0 pin (DIDR0) or AD15:8 pin (DIDR2)
  // and the digital input from this pin is not needed, this bit should
  // be written logic one to reduce power consumption in the digital
  // input buffer.
  if (enabled) {
    DIDR0 = 0;
    DIDR2 = 0;
  } else {
    DIDR0 = 0xFF;
    DIDR2 = 0xFF;
  }
}

// update buffer with new reading and increment to next channel
void ADCClass::updateMultiChannel() {
  // increment the channel index
  if (++channel_index_ == n_channels_) {
    channel_index_ = 0; // wrap around to zero
  }
  // set registers for next read
  _setChannel(channels_[channel_index_]);
  updateSingleChannel();
}

// update buffer with new reading (single channel)
void ADCClass::updateSingleChannel() {
  uint16_t value = ADC_RESULT;
  
  // check that we're not writing past the end of the buffer (possible if
  // main loop doesn't check finished() fast enough)
  if (current_index_ < buffer_len_) {
    if (!auto_trigger_on_) {
      next(); // start next conversion
    }
    if (ADMUX & (1 << ADLAR)) {
      buffer8_[current_index_++] = value >> 8;
    } else {
      buffer16_[current_index_++] = value;
    }    
  }
}

// call the registered callback function
void ADCClass::updateCallback() {
  // save the channel index (needed for callback)
  uint8_t channel_index = channel_index_;

  if (n_channels_ > 1) {

    // increment the channel index
    if (++channel_index_ == n_channels_) {
      channel_index_ = 0; // wrap around to zero
    }
    // set registers for next read
    _setChannel(channels_[channel_index_]);
  }
  uint16_t value = ADC_RESULT;

  // check that we're not writing past the end of the buffer (possible if
  // main loop doesn't check finished() fast enough)
  if (current_index_ < buffer_len_) {
    if (!auto_trigger_on_) {
      next(); // start next conversion
    }
    if (ADMUX & (1 << ADLAR)) {
      value = value >> 8;
    }
    (*callback_)(channel_index, value); // call the callback function
    current_index_++;
  }
}

// read 1.1V reference against AVcc and back-calculate AVcc
//
// Note that according to the ATmega2560 datasheet, the 1.1V
// reference is typically between 1.0 and 1.2 (+/- 9%).
//
// See: https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
float ADCClass::readVcc() {
  // save state of ADMUX and ADCSRB registers
  // ADMUX: REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
  uint8_t admux = ADMUX;
  // ADCSRB:  -  ACME  -   -  MUX5 ADTS2 ADTS1 ADTS0
  uint8_t adcsrb = ADCSRB;

  ADMUX &= ~(0x1F); // clear MUX4:0 on ADMUX register
  // set MUX4:0 to B11110
  ADMUX |= (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);

  // clear MUX5 bit on ADCSRB register
  ADCSRB &= ~(1 << MUX5);

  delay(3); // wait for Vref to settle

  ADCSRA |= (1 << ADSC); // start conversion
  while (bit_is_set(ADCSRA, ADSC)); // wait for conversion to complete
  float result = 1100.0 /(float)ADC_RESULT; // back-calculate AVcc in mV

  // reset ADMUX and ADCSRB registers to original state
  ADMUX = admux;
  ADCSRB = adcsrb;

  return result;
}

// interrupt service routine triggered when an ADC conversion completes
ISR(ADC_vect) {
  (*AdvancedADC.update)();
}

// interrupt service routine triggered when Timer1 overlfows
ISR(TIMER1_OVF_vect)
{

}

// Preinstantiate object
ADCClass AdvancedADC = ADCClass();

