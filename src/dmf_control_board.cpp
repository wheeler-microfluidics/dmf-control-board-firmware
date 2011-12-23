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
using namespace std;
#include <boost/date_time/posix_time/posix_time_types.hpp>
#else
#include "WProgram.h"
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <math.h>
#endif

#ifdef AVR
extern "C" {
  void PeakExceededWrapper();
}

const float DmfControlBoard::A0_SERIES_RESISTORS_[] = {1e5, 1e6};
const float DmfControlBoard::A1_SERIES_RESISTORS_[] = {1e3, 1e4}; // 1e5, 1e6};
const float DmfControlBoard::SAMPLING_RATES_[] = { 8908, 16611, 29253, 47458,
                                                 68191, 90293, 105263 };
const char DmfControlBoard::PROTOCOL_NAME_[] = "DMF Control Protocol";
const char DmfControlBoard::PROTOCOL_VERSION_[] = "0.1";
#else
const char DmfControlBoard::CSV_INDENT_[] = ",,,,,,,,";
#endif
const char DmfControlBoard::NAME_[] = "Arduino DMF Controller";
const char DmfControlBoard::MANUFACTURER_[] = "Wheeler Microfluidics Lab";
const char DmfControlBoard::HARDWARE_VERSION_[] = "1.2";
const char DmfControlBoard::SOFTWARE_VERSION_[] = ___SOFTWARE_VERSION___;
const char DmfControlBoard::URL_[] = "http://microfluidics.utoronto.ca/dmf_control_board";

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
  sprintf(log_message_string_,"command=0x%0X (%d)",
          cmd,cmd);
  LogMessage(log_message_string_, function_name);
#endif
  uint8_t return_code = RETURN_UNKNOWN_COMMAND;
  switch(cmd) {
#ifdef AVR // Commands that only the Arduino handles
    case CMD_GET_NUMBER_OF_CHANNELS:
      if(payload_length()==0) {
        Serialize(&number_of_channels_,sizeof(number_of_channels_));
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_STATE_OF_ALL_CHANNELS:
      if(payload_length()==0) {
        return_code = RETURN_OK;
        for(uint8_t chip=0; chip<number_of_channels_/40; chip++) {
          for(uint8_t port=0; port<5; port++) {
            Wire.beginTransmission(PCA9505_ADDRESS_+chip);
            Wire.send(PCA9505_OUTPUT_PORT_REGISTER_+port);
            Wire.endTransmission();
            Wire.requestFrom(PCA9505_ADDRESS_+chip,1);
            if (Wire.available()) {
              uint8_t data = Wire.receive();
              uint8_t state;
              for(uint8_t bit=0; bit<8; bit++) {
                state = (data >> bit & 0x01)==0;
                Serialize(&state, sizeof(state));
              }
            } else {
              return_code = RETURN_GENERAL_ERROR;
              break; break;
            }
          }
        }
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_STATE_OF_ALL_CHANNELS:
      if(payload_length()==number_of_channels_*sizeof(uint8_t)) {
        UpdateAllChannels();
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_STATE_OF_CHANNEL:
      if(payload_length()==sizeof(uint16_t)) {
        uint16_t channel = ReadUint16();
        if(channel>=number_of_channels_||channel<0) {
          return_code = RETURN_BAD_INDEX;
        } else {
          uint8_t chip = channel/40;
          uint8_t port = (channel%40)/8;
          uint8_t bit = (channel%40)%8;
          Wire.beginTransmission(PCA9505_ADDRESS_+chip);
          Wire.send(PCA9505_OUTPUT_PORT_REGISTER_+port);
          Wire.endTransmission();
          Wire.requestFrom(PCA9505_ADDRESS_+chip, 1);
          if(Wire.available()) {
            uint8_t data = Wire.receive();
            data = (data >> bit & 0x01)==0;
            Serialize(&data, sizeof(data));
            return_code = RETURN_OK;
          } else {
            return_code = RETURN_GENERAL_ERROR;
          }
        }
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_STATE_OF_CHANNEL:
      if(payload_length()==sizeof(uint16_t)+sizeof(uint8_t)) {
        uint16_t channel = ReadUint16();
        uint8_t state = ReadUint8();
        if(channel<number_of_channels_) {
          return_code = UpdateChannel(channel, state);
        } else {
          return_code = RETURN_BAD_INDEX;
        }
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_WAVEFORM:
      if(payload_length()==0) {
        return_code = RETURN_OK;
        uint8_t waveform = digitalRead(WAVEFORM_SELECT_);
        Serialize(&waveform, sizeof(waveform));
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WAVEFORM:
      if(payload_length()==sizeof(uint8_t)) {
        uint8_t waveform = ReadUint8();
        if(waveform==SINE || waveform==SQUARE) {
          digitalWrite(WAVEFORM_SELECT_, waveform);
          return_code = RETURN_OK;
        } else {
          return_code = RETURN_BAD_VALUE;
        }
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_WAVEFORM_VOLTAGE:
      if(payload_length()==0) {
        return_code = RETURN_OK;
        Serialize(&waveform_voltage_, sizeof(waveform_voltage_));
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WAVEFORM_VOLTAGE:
      if(payload_length()==sizeof(uint8_t)) {
        waveform_voltage_ = ReadUint8();
        SetPot(POT_INDEX_WAVEOUT_GAIN_2_, waveform_voltage_);
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_WAVEFORM_FREQUENCY:
      if(payload_length()==0) {
        return_code = RETURN_OK;
        Serialize(&waveform_frequency_, sizeof(waveform_frequency_));
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_WAVEFORM_FREQUENCY:
      if(payload_length()==sizeof(float)) {
        // the frequency of the LTC6904 oscillator needs to be set to 50x
        // the fundamental frequency
    	waveform_frequency_ = ReadFloat();
        float freq = waveform_frequency_*50;
        // valid frequencies are 1kHz to 68MHz
        if(freq<1e3 || freq>68e6) {
          return_code = RETURN_BAD_VALUE;
        } else {
          uint8_t oct = 3.322*log(freq/1039)/log(10);
          uint16_t dac = round(2048-(2078*pow(2, 10+oct))/freq);
          uint8_t cnf = 2; // CLK on, /CLK off
          // msb = OCT3 OCT2 OCT1 OCT0 DAC9 DAC8 DAC7 DAC6
          uint8_t msb = (oct << 4) | (dac >> 6);
          // lsb =  DAC5 DAC4 DAC3 DAC2 DAC1 DAC0 CNF1 CNF0
          uint8_t lsb = (dac << 2) | cnf;
          Wire.beginTransmission(LTC6904_);
          Wire.send(msb);
          Wire.send(lsb);
          Wire.endTransmission();     // stop transmitting
          return_code = RETURN_OK;
        }
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_SAMPLING_RATE:
      if(payload_length()==0) {
        Serialize(&SAMPLING_RATES_[sampling_rate_index_],sizeof(float));
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SAMPLING_RATE:
      if(payload_length()==sizeof(uint8_t)) {
        return_code = SetAdcPrescaler(ReadUint8());
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_SERIES_RESISTOR:
      if(payload_length()==sizeof(uint8_t)) {
        uint8_t channel = ReadUint8();
        return_code = RETURN_OK;
        switch(channel) {
          case 0:
            Serialize(&A0_SERIES_RESISTORS_[A0_series_resistor_index_],
                      sizeof(float));
            break;
          case 1:
            Serialize(&A1_SERIES_RESISTORS_[A1_series_resistor_index_],
                      sizeof(float));
            break;
          default:
            return_code = RETURN_BAD_INDEX;
            break;
        }
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SERIES_RESISTOR:
      if(payload_length()==2*sizeof(uint8_t)) {
        uint8_t channel = ReadUint8();
        return_code = SetSeriesResistor(channel, ReadUint8());
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SAMPLE_VOLTAGE:
      if(payload_length()<2*sizeof(uint8_t)+3*sizeof(uint16_t)) {
        return_code = RETURN_BAD_PACKET_SIZE;
      } else {
        uint16_t n_samples = ReadUint16();
        uint16_t n_sets = ReadUint16();
        uint16_t delay_between_sets_ms = ReadUint16();
        uint16_t n_channels = ReadUint8();
        if(n_samples*n_sets*n_channels>MAX_SAMPLES) {
          return_code = RETURN_GENERAL_ERROR;
        } else {
          if((payload_length()==sizeof(uint8_t)+3*sizeof(uint16_t)
             +n_channels*sizeof(uint8_t))
             || (payload_length()==sizeof(uint8_t)+3*sizeof(uint16_t)
             +n_channels*sizeof(uint8_t)
             +number_of_channels_*sizeof(uint8_t))) {
            return_code = RETURN_OK;

            // point the voltage_buffer_ to the payload_buffer_
            uint16_t* voltage_buffer_ = (uint16_t*)payload();
            // update the number of bytes written
            bytes_written(n_samples*n_sets*n_channels*sizeof(uint16_t));

            uint8_t channel[NUMBER_OF_ADC_CHANNELS];
            for(uint8_t i=0; i<n_channels; i++) {
              channel[i] = ReadUint8();
            }
            // update the channels (if they were included in the packet)
            if(payload_length()==sizeof(uint8_t)+3*sizeof(uint16_t)
               +n_channels*sizeof(uint8_t)
               +number_of_channels_*sizeof(uint8_t)){
              UpdateAllChannels();
            }
            // sample the voltages
            for(uint16_t i=0; i<n_sets; i++) {
              // dummy read to let the adc settle
              analogRead(channel[0]);
              for(uint16_t j=0; j<n_samples; j++) {
                for(uint16_t k=0; k<n_channels; k++) {
                  voltage_buffer_[i*n_samples*n_channels+j*n_channels+k] =
                    analogRead(channel[k]);
                }
              }
              uint32_t t = millis();
              while(millis()-t<delay_between_sets_ms) {
              }
            }
          } else {
            return_code = RETURN_BAD_PACKET_SIZE;
          }
        }
      }
      break;
    case CMD_MEASURE_IMPEDANCE:
      if(payload_length()<3*sizeof(uint16_t)) {
        return_code = RETURN_BAD_PACKET_SIZE;
      } else {
        uint16_t sampling_time_ms = ReadUint16();
        uint16_t n_samples = ReadUint16();
        uint16_t delay_between_samples_ms = ReadUint16();

        if(n_samples*2>MAX_SAMPLES) {
          return_code = RETURN_GENERAL_ERROR;
        } else {
          if(payload_length()==3*sizeof(uint16_t) ||
             (payload_length()==3*sizeof(uint16_t)
             +number_of_channels_*sizeof(uint8_t))) {
            return_code = RETURN_OK;

            // point the impedance_buffer_ to the payload_buffer_
            float* impedance_buffer = (float*)payload();

            // update the number of bytes written
            bytes_written(n_samples*2*sizeof(float));

            uint8_t original_A0_index = A0_series_resistor_index_;
            uint8_t original_A1_index = A1_series_resistor_index_;
            
            // set the resistors to their highest values
            SetSeriesResistor(0,
               sizeof(A0_SERIES_RESISTORS_)/sizeof(float)-1);
            SetSeriesResistor(1,
               sizeof(A1_SERIES_RESISTORS_)/sizeof(float)-1);

            // sample the actuation voltage
            uint32_t t_sample = millis();
            uint32_t t_delay;
            uint16_t hv_max = 0;
            uint16_t hv_min = 1024;
            uint16_t hv = 0;

            while(millis()-t_sample<sampling_time_ms) {
              hv = analogRead(0);

              // if the ADC is saturated, use a smaller resistor
              // and reset the peak
              if(hv==900) {
                if(A0_series_resistor_index_>0) {
                  SetSeriesResistor(0, \
                    A0_series_resistor_index_-1);
                  hv_max = 0;
                  hv_min = 1024;
                  continue;
                }
              }
              if(hv>hv_max) {
                hv_max = hv;
              }
              if(hv<hv_min) {
                hv_min = hv;
              }
            }

            if(return_code==RETURN_OK) {
              float V_hv = (float)(hv_max-hv_min)*5.0/1024.0*
                10e6/A0_SERIES_RESISTORS_[A0_series_resistor_index_];

              // update the channels (if they were included in the packet)
              if(payload_length()==3*sizeof(uint16_t)
                  +number_of_channels_*sizeof(uint8_t)){
                UpdateAllChannels();
              }

              // sample the feedback voltage
              for(uint16_t i=0; i<n_samples; i++) {
                uint16_t fb_max = 0;
                uint16_t fb_min = 1024;
                uint16_t fb = 0;
                t_sample = millis();

                while(millis()-t_sample<sampling_time_ms) {
                  fb = analogRead(1);

                  // if the ADC is saturated, use a smaller resistor
                  // and reset the peak
                  if(fb>900) {
                    if(A1_series_resistor_index_>0) {
                      SetSeriesResistor(1,
                        A1_series_resistor_index_-1);
                      fb_max = 0;
                      fb_min = 1024;
                      continue;
                    }
                  }
                  if(fb>fb_max) {
                    fb_max = fb;
                  }
                  if(fb<fb_min) {
                    fb_min = fb;
                  }
                }

                float Z_fb = A1_SERIES_RESISTORS_[
                                A1_series_resistor_index_];
                float V_fb = (float)(fb_max-fb_min)*5.0/1024.0/2*sqrt(0.5);

                impedance_buffer[2*i] = V_fb;
                impedance_buffer[2*i+1] = Z_fb;

                t_delay = millis();
                while(millis()-t_delay<delay_between_samples_ms) {
                }
              }
            }
            
            // set the resistors back to their original states            
            SetSeriesResistor(0, original_A0_index);
            SetSeriesResistor(1, original_A1_index);
          } else {
            return_code = RETURN_BAD_PACKET_SIZE;
          }
        }
      }
      break;
    case CMD_GET_PEAK_VOLTAGE:
      if(payload_length()<sizeof(uint8_t)+sizeof(uint16_t)) {
        return_code = RETURN_BAD_PACKET_SIZE;
      } else {
        uint8_t adc_channel = ReadUint8();
        uint16_t sampling_time_ms = ReadUint16();
        if(adc_channel<2) {
          float peak;
          uint8_t interrupt;
          if(adc_channel==0) {
        	interrupt = HV_PEAK_INTERRUPT_;
          } else {
          	interrupt = FB_PEAK_INTERRUPT_;
          }
          peak = GetPeakVoltage(interrupt, sampling_time_ms);
          Serialize(&peak,sizeof(peak));
          return_code = RETURN_OK;
        } else {
          return_code = RETURN_GENERAL_ERROR;
        }
      }
      break;
#endif
    default:
      return_code = RemoteObject::ProcessCommand(cmd);
#ifndef AVR
      if(return_code == RETURN_UNKNOWN_COMMAND) {
        LogError("Unrecognized command", function_name);
      }
#endif
      break;
  }
  return return_code;
}

#ifdef AVR
////////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the Arduino.
//
////////////////////////////////////////////////////////////////////////////////

void DmfControlBoard::begin() {
  RemoteObject::begin();
  analogReference(EXTERNAL);

  pinMode(AD5204_SLAVE_SELECT_PIN_, OUTPUT);
  pinMode(A0_SERIES_RESISTOR_0_, OUTPUT);
  pinMode(A1_SERIES_RESISTOR_0_, OUTPUT);
  pinMode(A1_SERIES_RESISTOR_1_, OUTPUT);
  pinMode(A1_SERIES_RESISTOR_2_, OUTPUT);
  pinMode(WAVEFORM_SELECT_, OUTPUT);

  // versions > 1.1 need to pull a pin low to turn on the power supply
  pinMode(PWR_SUPPLY_ON_, OUTPUT);
  digitalWrite(PWR_SUPPLY_ON_, LOW);

  Serial.begin(DmfControlBoard::BAUD_RATE);

  Serial.print(name());
  Serial.print(" v");
  Serial.println(hardware_version());
  Serial.print("Firmware version: ");
  Serial.println(software_version());

  // Check how many switching boards are connected.  Each additional board's
  // address must equal the previous boards address +1 to be valid.
  number_of_channels_ = 0;
  for(uint8_t chip=0; chip<8; chip++) {
    Wire.beginTransmission(PCA9505_ADDRESS_+chip);
    Wire.send(PCA9505_CONFIG_IO_REGISTER_);
    Wire.endTransmission();
    Wire.requestFrom(PCA9505_ADDRESS_+chip,1);
    if (Wire.available()) {
      Wire.receive();
      if(number_of_channels_==40*chip) {
        number_of_channels_ = 40*(chip+1);
      }
      Serial.print("HV board ");
      Serial.print((int)chip);
      Serial.println(" connected.");
      // set all PCA0505 ports in output mode and initialize to ground
      for(uint8_t port=0; port<5; port++) {
        SendI2C(PCA9505_ADDRESS_+chip, PCA9505_CONFIG_IO_REGISTER_+port, 0x00);
        SendI2C(PCA9505_ADDRESS_+chip, PCA9505_OUTPUT_PORT_REGISTER_+port, 0xFF);
      }
    }
  }
  Serial.print(number_of_channels_);
  Serial.println(" channels available.");

  // set waveform (SINE=0, SQUARE=1)
  digitalWrite(WAVEFORM_SELECT_, SINE);

  // sets the maximum output voltage for the waveform generator
  uint8_t waveout_gain_1 = 112;

  // sets the value of the analog reference
  uint8_t aref = 255;

  // sets the value of the virtual ground
  uint8_t vgnd = 124;

  // if the EEPROM_INIT flag has been set, replace default values with those
  // stored in EEPROM
  if(EEPROM.read(EEPROM_INIT)==0) {
	Serial.println("Using calibration info from EEPROM.");
    waveout_gain_1 = EEPROM.read(EEPROM_POT_WAVEOUT_GAIN_1);
    aref = EEPROM.read(EEPROM_AREF);
    vgnd = EEPROM.read(EEPROM_VGND);
  } else {
    Serial.println("Using default calibration info.");
  }
  Serial.print("waveout_gain_1=");
  Serial.println(waveout_gain_1, DEC);
  Serial.print("aref=");
  Serial.println(aref, DEC);
  Serial.print("vgnd=");
  Serial.println(vgnd, DEC);

  // set all digital pots
  SetPot(POT_INDEX_AREF_, aref);
  SetPot(POT_INDEX_VGND_, vgnd);
  SetPot(POT_INDEX_WAVEOUT_GAIN_1_, waveout_gain_1);
  SetPot(POT_INDEX_WAVEOUT_GAIN_2_, 0);

  SetSeriesResistor(0, 0);
  SetSeriesResistor(1, 0);
  SetAdcPrescaler(4);
}

void DmfControlBoard::PeakExceeded() {
  peak_++;
  SetPot(POT_INDEX_AREF_, peak_);
}

// send a command and some data to one of the I2C chips
// (code based on http://gdallaire.net/blog/?p=18)
void DmfControlBoard::SendI2C(uint8_t row, uint8_t cmd, uint8_t data) {
  Wire.beginTransmission(row);
  Wire.send(cmd);
  Wire.send(data);
  Wire.endTransmission();
}

void DmfControlBoard::SendSPI(uint8_t pin, uint8_t address, uint8_t data) {
  digitalWrite(pin, LOW);
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(pin, HIGH);
}

uint8_t DmfControlBoard::SetPot(uint8_t index, uint8_t value) {
  if(index>=0 && index<4) {
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
  if(return_code==RETURN_OK){
    sampling_rate_index_ = index;
  }
  return return_code;
}


uint8_t DmfControlBoard::SetSeriesResistor(const uint8_t channel,
                                           const uint8_t index) {
  uint8_t return_code = RETURN_OK;
  if(channel==0) {
    switch(index) {
      case 0:
        digitalWrite(A0_SERIES_RESISTOR_0_, HIGH);
        break;
      case 1:
        digitalWrite(A0_SERIES_RESISTOR_0_, LOW);
        break;
      default:
        return_code = RETURN_BAD_INDEX;
        break;
    }
    if(return_code==RETURN_OK) {
      A0_series_resistor_index_ = index;
    }
  } else if(channel==1) {
    switch(index) {
      case 0:
        digitalWrite(A1_SERIES_RESISTOR_0_, HIGH);
        digitalWrite(A1_SERIES_RESISTOR_1_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_2_, LOW);
        break;
      case 1:
        digitalWrite(A1_SERIES_RESISTOR_0_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_1_, HIGH);
        digitalWrite(A1_SERIES_RESISTOR_2_, LOW);
        break;
      /* these 2 resistors are unreliable
      case 2:
        digitalWrite(A1_SERIES_RESISTOR_0_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_1_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_2_, HIGH);
        break;
      case 3:
        digitalWrite(A1_SERIES_RESISTOR_0_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_1_, LOW);
        digitalWrite(A1_SERIES_RESISTOR_2_, LOW);
        break;
      */
      default:
        return_code = RETURN_BAD_INDEX;
        break;
    }
    if(return_code==RETURN_OK) {
      A1_series_resistor_index_ = index;
      // wait for pot to settle
      delayMicroseconds(200);
    }
  } else { // bad channel
    return_code = RETURN_BAD_INDEX;
  }
  return return_code;
}

float DmfControlBoard::GetPeakVoltage(const uint8_t interrupt,
                               	   	  const uint16_t sample_time_ms) {
  peak_ = 128;
  SetPot(POT_INDEX_AREF_, peak_);
  attachInterrupt(interrupt, PeakExceededWrapper, RISING);
  delay(sample_time_ms);
  detachInterrupt(interrupt);
  SetPot(POT_INDEX_AREF_, 255);
  return (float)peak_/255*5.0;
}

// update the state of all channels
void DmfControlBoard::UpdateAllChannels() {
  // Each PCA9505 chip has 5 8-bit output registers for a total of 40 outputs
  // per chip. We can have up to 8 of these chips on an I2C bus, which means
  // we can control up to 320 channels.
  //   Each register represent 8 channels (i.e. the first register on the
  // first PCA9505 chip stores the state of channels 0-7, the second register
  // represents channels 8-15, etc.).
  uint8_t data = 0;
  for(uint8_t chip=0; chip<number_of_channels_/40; chip++) {
    for(uint8_t port=0; port<5; port++) {
      data = 0;
      for(uint8_t i=0; i<8; i++) {
        data += (ReadUint8()==0)<<i;
      }
      SendI2C(PCA9505_ADDRESS_+chip, PCA9505_OUTPUT_PORT_REGISTER_+port, data);
    }
  }
}

// Update the state of single channel.
// Note: Do not use this function in a loop to update all channels. If you
//       want to update all channels, use the UpdateAllChannels function
//       instead because it will be 8x more efficient.
uint8_t DmfControlBoard::UpdateChannel(const uint16_t channel,
                                       const uint8_t state) {
  uint8_t data = 0;
  uint8_t chip = channel/40;
  uint8_t port = (channel%40)/8;
  uint8_t bit = (channel%40)%8;
  Wire.beginTransmission(PCA9505_ADDRESS_+chip);
  Wire.send(PCA9505_OUTPUT_PORT_REGISTER_+port);
  Wire.endTransmission();
  Wire.requestFrom(PCA9505_ADDRESS_+chip,1);
  if (Wire.available()) {
    uint8_t data = Wire.receive();
    bitWrite(data, bit, state==0);
    SendI2C(PCA9505_ADDRESS_+chip,
            PCA9505_OUTPUT_PORT_REGISTER_+port,
            data);
    return RETURN_OK;
  } else {
    return RETURN_GENERAL_ERROR;
  }
}

#else
////////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the PC.
//
////////////////////////////////////////////////////////////////////////////////

uint8_t DmfControlBoard::SendCommand(const uint8_t cmd) {
  const char* function_name = "SendCommand()";
  std::ostringstream msg;
  msg << "time since last," << MillisecondsSinceLastCheck();
  RemoteObject::SendCommand(cmd);
  msg << ",ms,returned in," << MillisecondsSinceLastCheck() << ",ms,command,"
      << (int)cmd << ",";
  if(return_code()!=RETURN_OK) {
    msg << "return code," << (int)return_code();
  }
  LogExperiment(msg.str().c_str());
  msg.str("");
  msg << "returned " << (int)return_code();
  LogMessage(msg.str().c_str(), function_name);
  return return_code();
}

uint16_t DmfControlBoard::number_of_channels() {
  const char* function_name = "number_of_channels()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(SendCommand(CMD_GET_NUMBER_OF_CHANNELS)==RETURN_OK) {
    if(payload_length()==sizeof(uint16_t)) {
      uint16_t number_of_channels = ReadUint16();
      sprintf(log_message_string_,
              "number_of_channels=%d",
              number_of_channels);
      LogMessage(log_message_string_, function_name);
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
  if(SendCommand(CMD_GET_STATE_OF_ALL_CHANNELS)==RETURN_OK) {
    LogMessage("CMD_GET_STATE_OF_ALL_CHANNELS", function_name);
    std::vector<uint8_t> state_of_channels;
    for(int i=0; i<payload_length(); i++) {
      state_of_channels.push_back(ReadUint8());
      sprintf(log_message_string_,
              "state_of_channels_[%d]=%d",
              i,state_of_channels[i]);
      LogMessage(log_message_string_, function_name);
    }
    return state_of_channels;
  }
  return std::vector<uint8_t>(); // return an empty vector
};

uint8_t DmfControlBoard::state_of_channel(const uint16_t channel) {
  const char* function_name = "state_of_channel()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel,sizeof(channel));
  if(SendCommand(CMD_GET_STATE_OF_CHANNEL)==RETURN_OK) {
    LogMessage("CMD_GET_STATE_OF_CHANNEL", function_name);
    if(payload_length()==sizeof(uint8_t)) {
      uint8_t state = ReadUint8();
      sprintf(log_message_string_, "state=%d", state);
      LogMessage(log_message_string_, function_name);
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
  if(SendCommand(CMD_GET_SAMPLING_RATE)==RETURN_OK) {
    LogMessage("CMD_GET_SAMPLING_RATE", function_name);
    if(payload_length()==sizeof(float)) {
      float sampling_rate = ReadFloat();
      sprintf(log_message_string_,
              "sampling_rate_=%.1e",sampling_rate);
      LogMessage(log_message_string_, function_name);
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
  Serialize(&sampling_rate,sizeof(sampling_rate));
  if(SendCommand(CMD_SET_SAMPLING_RATE)==RETURN_OK) {
    LogMessage("CMD_SET_SAMPLING_RATE", function_name);
    LogMessage("sampling rate set successfully", function_name);
    std::ostringstream msg;
    msg << "set sampling rate," << (int)sampling_rate << endl;
    LogExperiment(msg.str().c_str());
  }
  return return_code();
}

float DmfControlBoard::series_resistor(const uint8_t channel) {
  const char* function_name = "series_resistor()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel,sizeof(channel));
  if(SendCommand(CMD_GET_SERIES_RESISTOR)==RETURN_OK) {
    LogMessage("CMD_GET_SERIES_RESISTOR", function_name);
    if(payload_length()==sizeof(float)) {
      float series_resistor = ReadFloat();
      sprintf(log_message_string_,
              "series_resistor_=%.1e",series_resistor);
      LogMessage(log_message_string_, function_name);
      return series_resistor;
    } else {
      LogMessage("CMD_GET_SERIES_RESISTOR, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

std::string DmfControlBoard::waveform() {
  const char* function_name = "waveform()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(SendCommand(CMD_GET_WAVEFORM)==RETURN_OK) {
    LogMessage("CMD_GET_WAVEFORM", function_name);
    if(payload_length()==1) {
      uint8_t waveform = ReadUint8();
      std::string waveform_str;
      if(waveform==SINE || waveform==SQUARE) {
        if(waveform==SINE) {
          waveform_str = "SINE";
        } else if(waveform==SQUARE) {
          waveform_str = "SQUARE";
        }
        std::ostringstream msg;
        msg << "waveform=" << waveform_str;
        LogMessage(msg.str().c_str(), function_name);
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
  if(SendCommand(CMD_GET_WAVEFORM_VOLTAGE)==RETURN_OK) {
    LogMessage("CMD_GET_WAVEFORM_VOLTAGE", function_name);
    if(payload_length()==sizeof(uint8_t)) {
      float v_rms = (float)ReadUint8()/255.0*4.0/2*sqrt(0.5);
      std::ostringstream msg;
      msg << "waveform_voltage=" << v_rms;
      LogMessage(msg.str().c_str(), function_name);
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
  if(SendCommand(CMD_GET_WAVEFORM_FREQUENCY)==RETURN_OK) {
    LogMessage("CMD_GET_WAVEFORM_FREQUENCY", function_name);
    if(payload_length()==sizeof(float)) {
      float freq_hz = ReadFloat();
      std::ostringstream msg;
      msg << "waveform_frequency=" << freq_hz;
      LogMessage(msg.str().c_str(), function_name);
      return freq_hz;
    } else {
      LogMessage("CMD_GET_WAVEFORM_FREQUENCY, Bad packet size",
                 function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

uint8_t DmfControlBoard::set_series_resistor(const uint8_t channel,
                                           const uint8_t series_resistor) {
  const char* function_name = "set_series_resistor()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel,sizeof(channel));
  Serialize(&series_resistor,sizeof(series_resistor));
  if(SendCommand(CMD_SET_SERIES_RESISTOR)==RETURN_OK) {
    LogMessage("CMD_SET_SERIES_RESISTOR", function_name);
    LogMessage("series resistor set successfully", function_name);
    std::ostringstream msg;
    msg << "set series resistor," << (int)channel
        << (int)series_resistor << endl;
    LogExperiment(msg.str().c_str());
  }
  return return_code();
}

uint8_t DmfControlBoard::set_state_of_all_channels(const vector<uint8_t> state) {
  const char* function_name = "set_state_of_all_channels()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&state[0],state.size()*sizeof(uint8_t));
  if(SendCommand(CMD_SET_STATE_OF_ALL_CHANNELS)==RETURN_OK) {
    LogMessage("CMD_SET_STATE_OF_ALL_CHANNELS", function_name);
    LogMessage("all channels set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_state_of_channel(const uint16_t channel, const uint8_t state) {
  const char* function_name = "set_state_of_channel()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&channel,sizeof(channel));
  Serialize(&state,sizeof(state));
  if(SendCommand(CMD_SET_STATE_OF_CHANNEL)==RETURN_OK) {
    LogMessage("CMD_SET_STATE_OF_CHANNEL", function_name);
    LogMessage("channel set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_waveform(bool waveform) {
  const char* function_name = "set_waveform()";
  std::ostringstream msg;
  LogSeparator();
  LogMessage("send command", function_name);
  uint8_t data = waveform;
  Serialize(&data,sizeof(data));
  if(SendCommand(CMD_SET_WAVEFORM)==RETURN_OK) {
    LogMessage("CMD_SET_WAVEFORM", function_name);
    LogMessage("waveform set successfully", function_name);
  }
  return return_code();
}

uint8_t DmfControlBoard::set_waveform_voltage(const float v_rms){
  const char* function_name = "set_waveform_voltage()";
  std::ostringstream msg;
  LogSeparator();
  LogMessage("send command", function_name);
  // max voltage is 4 Vpk-pk (1.414 Vrms)
  if(v_rms>=0 && v_rms<=2*sqrt(.5)) {
    uint8_t data = v_rms*sqrt(2)/2*255;
    Serialize(&data,sizeof(data));
    msg << "data=" << (int)data;
    LogMessage(msg.str().c_str(), function_name);
    if(SendCommand(CMD_SET_WAVEFORM_VOLTAGE)==RETURN_OK) {
      LogMessage("CMD_SET_WAVEFORM_VOLTAGE", function_name);
      LogMessage("volage set successfully", function_name);
      msg.str("");
      msg << "set_actuation_voltage," << v_rms << ",Vrms" << endl;
      LogExperiment(msg.str().c_str());
    }
  } else {
    return_code_ = RETURN_BAD_VALUE;
    throw runtime_error("Value out of bounds.");
  }
  return return_code();
}

uint8_t DmfControlBoard::set_waveform_frequency(const float freq_hz) {
  const char* function_name = "set_actuation_frequency()";
  LogSeparator();
  sprintf(log_message_string_,"freq_hz=%.1f",freq_hz);
  LogMessage(log_message_string_, function_name);
  LogMessage("send command", function_name);
  Serialize(&freq_hz,sizeof(freq_hz));
  if(SendCommand(CMD_SET_WAVEFORM_FREQUENCY)==RETURN_OK) {
    LogMessage("CMD_SET_WAVEFORM_FREQUENCY", function_name);
    LogMessage("frequency set successfully", function_name);
    std::ostringstream msg;
    msg << "set_actuation_frequency," << freq_hz/1000 << ",kHz" << endl;
    LogExperiment(msg.str().c_str());
  }
  return return_code();
}

std::vector<uint16_t> DmfControlBoard::SampleVoltage(
                                        std::vector<uint8_t> adc_channel,
                                        uint16_t n_samples,
                                        uint16_t n_sets,
                                        uint16_t delay_between_sets_ms,
                                        const std::vector<uint8_t> state) {
  const char* function_name = "SampleVoltage()";
  LogSeparator();
  LogMessage("send command", function_name);
  // if we get this far, everything is ok
  Serialize(&n_samples,sizeof(n_samples));
  Serialize(&n_sets,sizeof(n_sets));
  Serialize(&delay_between_sets_ms,sizeof(delay_between_sets_ms));
  uint8_t n_channels = adc_channel.size();
  Serialize(&n_channels,sizeof(n_channels));
  for(uint8_t i=0; i<adc_channel.size(); i++) {
    Serialize(&adc_channel[i],sizeof(adc_channel[i]));
  }
  std::ostringstream msg;
  msg << "SampleVoltage,";

  msg << endl << CSV_INDENT_ << "n_samples,"
      << (int)n_samples << endl << CSV_INDENT_ << "n_sets," << (int)n_sets
      << endl << CSV_INDENT_ << "delay_between_sets_ms,"
      << (int)delay_between_sets_ms << endl;
  if(SendCommand(CMD_SAMPLE_VOLTAGE)==RETURN_OK) {
    LogMessage("CMD_SAMPLE_VOLTAGE", function_name);
    uint16_t n_samples = payload_length()/sizeof(uint16_t);
    sprintf(log_message_string_,"Read %d feedback samples",n_samples);
    LogMessage(log_message_string_,function_name);
    std::vector<uint16_t> voltage_buffer(n_samples);
    for(uint16_t i=0; i<n_samples; i++) {
      voltage_buffer[i] = ReadUint16();
    }
    for(int i=0; i<adc_channel.size(); i++) {
      for(int j=0; j<n_sets; j++) {
        msg << CSV_INDENT_ << "voltage_buffer_[" << i << "][" << j << "],";
        // calculate the DC bias
        double dc_bias = 0;
        for(int k=0; k<n_samples; k++) {
          dc_bias += (float)voltage_buffer[j*adc_channel.size()*n_samples+
                                           k*adc_channel.size()+i]
                                           /1024*5/n_samples;
          msg << (float)voltage_buffer[j*adc_channel.size()*n_samples+
                                       k*adc_channel.size()+i]/1024*5 << ",";
        }
        double v_rms = 0;
        for(int k=0; k<n_samples; k++) {
          v_rms += pow((float)voltage_buffer[j*adc_channel.size()*n_samples+
                                             k*adc_channel.size()+i]
                                             /1024*5-dc_bias,2)/n_samples;
        }
        v_rms = sqrt(v_rms);
        msg << endl << CSV_INDENT_ << "dc_bias," << dc_bias << endl
            << CSV_INDENT_ << "v_rms," << v_rms << endl;
      }
    }
    LogExperiment(msg.str().c_str());
    return voltage_buffer;
  }
  return std::vector<uint16_t>(); // return an empty vector
}

std::vector<float> DmfControlBoard::MeasureImpedance(
                                          uint16_t sampling_time_ms,
                                          uint16_t n_samples,
                                          uint16_t delay_between_samples_ms,
                                          const std::vector<uint8_t> state) {
  const char* function_name = "MeasureImpedance()";
  LogSeparator();
  LogMessage("send command", function_name);
  // if we get this far, everything is ok
  Serialize(&sampling_time_ms,sizeof(sampling_time_ms));
  Serialize(&n_samples,sizeof(n_samples));
  Serialize(&delay_between_samples_ms,sizeof(delay_between_samples_ms));
  Serialize(&state[0],state.size()*sizeof(uint8_t));
  std::ostringstream msg;
  msg << "MeasureImpedance,";
  msg << endl << CSV_INDENT_ << "sampling_time_ms," << (int)sampling_time_ms
      << "n_samples," << (int)n_samples << endl << CSV_INDENT_
      << "delay_between_samples_ms," << (int)delay_between_samples_ms << endl;
  if(SendCommand(CMD_MEASURE_IMPEDANCE)==RETURN_OK) {
    LogMessage("CMD_MEASURE_IMPEDANCE", function_name);
    uint16_t n_samples = payload_length()/2/sizeof(float);
    sprintf(log_message_string_,"Read %d impedance samples",n_samples);
    LogMessage(log_message_string_,function_name);
    std::vector<float> impedance_buffer(2*n_samples);
    for(uint16_t i=0; i<2*n_samples; i++) {
      impedance_buffer[i] = ReadFloat();
    }
    LogExperiment(msg.str().c_str());
    return impedance_buffer;
  }
  return std::vector<float>(); // return an empty vector
}

float DmfControlBoard::GetPeakVoltage(uint8_t adc_channel,
                                      uint16_t sampling_time_ms) {
  const char* function_name = "GetPeakVoltage()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&adc_channel,sizeof(adc_channel));
  Serialize(&sampling_time_ms,sizeof(sampling_time_ms));
  if(SendCommand(CMD_GET_PEAK_VOLTAGE)==RETURN_OK) {
    if(payload_length()==sizeof(float)) {
      return ReadFloat();
    } else {
      LogError("Bad packet size", function_name);
      throw runtime_error("Bad packet size.");
    }
  }
  return 0;
}

uint8_t DmfControlBoard::SetExperimentLogFile(const char* file_name) {
  const char* function_name = "SetExperimentLogFile()";
  if(experiment_log_file_.is_open()) {
    experiment_log_file_.close();
  }
  std::ostringstream msg;
  msg << "file_name=" << file_name;
  LogMessage(msg.str().c_str(), function_name);
  experiment_log_file_name_ = file_name;
  t_last_check_ = boost::posix_time::microsec_clock::universal_time();
  experiment_log_file_.open(experiment_log_file_name_.c_str(), ios::app);
  if(experiment_log_file_.fail()==false) {
    return RETURN_OK;
  } else {
    throw runtime_error("Error creating log file.");
  }
}

void DmfControlBoard::LogExperiment(const char* msg) {
  if(experiment_log_file_.is_open()) {
    experiment_log_file_ << msg;
    experiment_log_file_.flush();
  }
}

float DmfControlBoard::MillisecondsSinceLastCheck() {
  boost::posix_time::ptime t = t_last_check_;
  t_last_check_ = boost::posix_time::microsec_clock::universal_time();
  return floor(0.5+(float)(boost::posix_time::microsec_clock::universal_time()-t)
         .total_microseconds()/1000);
}

#endif // AVR

