/*
Copyright 2011 Ryan Fobel, 2013 Christian Fobel

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

#include "RemoteObject.h"

#if defined(AVR)
  #include <util/crc16.h>
  #include <AdvancedADC.h>
#endif
#if defined(AVR) || defined(__SAM3X8E__)
  #include "Arduino.h"
  #include <Wire.h>
  #include <SPI.h>
  #include <OneWire.h>
  #ifdef AVR
    #include <AdvancedADC.h>
    #include <EEPROM.h>
  #elif defined(__SAM3X8E__)
    #include <DueFlashStorage.h>
    extern DueFlashStorage EEPROM;
  #endif
  extern "C" void __cxa_pure_virtual(void); // These declarations are needed for
  void __cxa_pure_virtual(void) {}          // virtual functions on the Arduino.
#else
  #include <string>
  #include <boost/thread.hpp>
  #include <boost/timer.hpp>
  #include <boost/date_time/posix_time/posix_time_types.hpp>
  #include <boost/format.hpp>
  #include <exception>
  using namespace std;
  using boost::format;
#endif


#ifdef __SAM3X8E__
  const char RemoteObject::MCU_TYPE_[] = "SAM3X8E";
#else
  const char RemoteObject::MCU_TYPE_[] = "ATmega2560";
#endif // __SAM3X8E__

RemoteObject::RemoteObject(bool crc_enabled
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
                           ,const char* class_name //used for logging
#endif
                           ) : crc_enabled_(crc_enabled)
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
                               ,class_name_(class_name)
#endif
                               {
    bytes_received_ = 0;
    un_escaping_ = false;
    payload_length_ = 0;
    bytes_read_ = 0;
    bytes_written_ = 0;
    debug_ = false;

#if defined(AVR) || defined(__SAM3X8E__)
    // Initialize pin mode and state of digital pins from persistent storage
    // _(i.e., EEPROM on AVR)_.
    for (uint8_t i = 0; i <= 54 / 8; i++) {
        uint8_t mode = persistent_read(PERSISTENT_PIN_MODE_ADDRESS + i);
        uint8_t state = persistent_read(PERSISTENT_PIN_STATE_ADDRESS + i);
        for (uint8_t j = 0; j < 8; j++) {
            if (i * 8 + j < 54) {
                pinMode(i * 8 + j, (~mode >> j) & 0x01);
                digitalWrite(i * 8 + j, (~state >> j) & 0x01);
            }
        }
    }
#endif
}

RemoteObject::~RemoteObject() {}

void RemoteObject::send_byte(const uint8_t b) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
    const char* function_name = "send_byte()";
#endif
    if (b == FRAME_BOUNDARY || b == CONTROL_ESCAPE) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
        log_message(str(format("write escape (0x%0X)") % (int)b).c_str(),
                    function_name);
#endif
        Serial.write(CONTROL_ESCAPE);
        Serial.write(b ^ ESCAPE_XOR);
    } else {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
        log_message(str(format("write (0x%0X)") % (int)b).c_str(),
                    function_name);
#endif
        Serial.write(b);
    }
}

uint16_t RemoteObject::update_crc(uint16_t crc, uint8_t data) {
#if defined(AVR)
  crc = _crc16_update(crc,data);
#else
  crc ^= data;
  for (uint8_t i=0; i<8; i++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
#endif
  return crc;
}

void RemoteObject::send_preamble(const uint8_t cmd) {
  payload_length_ = bytes_written_;
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  const char* function_name = "send_preamble()";
  log_message(str(format("command=0x%0X (%d), payload_length=%d") %
    (int)cmd % (int)cmd % payload_length_).c_str(), function_name);
#endif
  Serial.write(FRAME_BOUNDARY);
  if (crc_enabled_) {
    tx_crc_ = 0xFFFF; // reset crc
    tx_crc_ = update_crc(tx_crc_, cmd);
  }
  send_byte(cmd);
  if (payload_length_<128) {
    if (crc_enabled_) {
      tx_crc_ = update_crc(tx_crc_, (uint8_t)payload_length_);
    }
    send_byte((uint8_t)payload_length_);
  } else {
    if (crc_enabled_) {
      tx_crc_ = update_crc(tx_crc_, (uint8_t)((0x8000|payload_length_)>>8));
      tx_crc_ = update_crc(tx_crc_, (uint8_t)payload_length_);
    }
    send_byte((uint8_t)((0x8000|payload_length_)>>8));
    send_byte((uint8_t)payload_length_);
  }
}

uint8_t RemoteObject::send_command(const uint8_t cmd) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  const char* function_name = "send_command()";
  log_separator();
  log_message("",function_name);
#endif
  packet_cmd_ = cmd;
  send_preamble(cmd);
  send_payload();
  waiting_for_reply_to_ = cmd;
  return_code_ = wait_for_reply();
  return return_code_;
}

void RemoteObject::send_non_blocking_command(const uint8_t cmd) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  const char* function_name = "send_non_blocking_command()";
  log_separator();
  log_message("", function_name);
#endif
  packet_cmd_ = cmd;
  send_preamble(cmd);
  send_payload();
  waiting_for_reply_to_ = cmd;
}

void RemoteObject::send_interrupt() {
  // send a dummy byte
  send_byte(0);
}

void RemoteObject::serialize(const uint8_t* u,const uint16_t size) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  const char* function_name = "serialize()";
  log_message(str(format("%d bytes.") % size).c_str(), function_name);
#endif
  //TODO check that MAX_PAYLOAD_LENGTH isn't exceeded
  for (uint16_t i = 0; i < size; i++) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
    log_message(str(format("(0x%0X) byte %d") % int(u[i]) % i).c_str(),
      function_name);
#endif
    payload_[bytes_written_+i] = u[i];
  }
  bytes_written_ += size;
}

void RemoteObject::send_payload() {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  const char* function_name = "send_payload()";
  log_message(str(format("%d bytes") % payload_length_).c_str(), function_name);
#endif
  for (uint16_t i = 0; i < payload_length_; i++) {
    if (crc_enabled_) {
      tx_crc_ = update_crc(tx_crc_, payload_[i]);
    }
    send_byte(payload_[i]);
  }
  if (crc_enabled_) {
    send_byte((uint8_t)tx_crc_);
    send_byte((uint8_t)(tx_crc_>>8));
  }
  payload_length_ = 0;
  bytes_written_ = 0;
}

void RemoteObject::send_reply(uint8_t return_code) {
  serialize(&return_code,1);
  send_preamble(packet_cmd_);
  send_payload();
}

const char* RemoteObject::read_string() {
  // TODO check that we're not reading past the end of the buffer
  uint8_t length = strlen((const char*)payload_)+1;
  bytes_read_ += length;
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  const char* function_name = "read_string()";
  log_message(str(format("=\"%s\", bytes_read_=%d") %
    (const char*)(payload_+bytes_read_-length) % bytes_read_).c_str(),
    function_name);
#endif
  return (const char*)(payload_+bytes_read_-length);
}

uint8_t RemoteObject::wait_for_reply() {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  const char* function_name = "wait_for_reply()";
  log_message("", function_name);
  time_cmd_sent_ = boost::posix_time::microsec_clock::universal_time();
  uint8_t cmd = packet_cmd_;
#endif
  while (waiting_for_reply_to_) {
    listen();
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
    if ((boost::posix_time::microsec_clock::universal_time()
       -time_cmd_sent_).total_milliseconds()>TIMEOUT_MILLISECONDS) {
      return_code_ = RETURN_TIMEOUT;
      throw runtime_error(str(format("Command 0x%0X (%d) timeout.") %
        (int)cmd % (int)cmd).c_str());
    }
#endif
  }
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  log_message(str(format("return code=%d, cmd returned in %d us") %
    (int)return_code_ % (boost::posix_time::microsec_clock::universal_time()
    -time_cmd_sent_).total_microseconds()).c_str(), function_name);
  if (return_code_!=RETURN_OK) {
    throw runtime_error(str(format("Error sending command 0x%0X (%d). "
        "Return code=%d.") % (int)cmd % (int)cmd % (int)return_code_).c_str());
  }
#endif
  return return_code_;
}

uint8_t RemoteObject::validate_reply(const uint8_t cmd) {
  if (wait_for_reply() == RETURN_OK) {
    if (cmd!=(packet_cmd_^0x80)) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
      throw runtime_error(str(format("Requesting for data from command 0x%0X "
        "(%d), but the previously sent command was 0x%0X (%d).") % int(cmd) %
        int(cmd) % int(packet_cmd_) % int(packet_cmd_)).c_str());
#endif
      return RETURN_GENERAL_ERROR;

    // if we've previously read bytes from the buffer, throw an error
    } else if (bytes_read()) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
      throw runtime_error("Data from this command has already been retrieved.");
#endif
      return RETURN_GENERAL_ERROR;
    }
  }
  return return_code_;
}

void RemoteObject::process_packet() {
    if (packet_cmd_ & 0x80) {
        /* Commands have MSB == 1, so this packet contains a command.  Start
         * preparing the response and process the command. */

        // Flip the MSB for reply
        packet_cmd_ = packet_cmd_ ^ 0x80;
        return_code_ = RETURN_UNKNOWN_COMMAND;
        process_command(packet_cmd_ ^ 0x80);
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
        log_separator();
#endif
    } else {
        // This packet does not contain a command, so assume it is a response.
        return_code_ = payload_[payload_length_ - 1];
        /* Decrement the payload-length because we've already read the return
         * code (which was included in the payload-length). */
        payload_length_--;
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
        const char* function_name = "process_packet()";
        log_message(str(format("(0x%0X). This packet is a reply to command "
                              "(%d)") % (packet_cmd_ ^ 0x80) %
                              (packet_cmd_^0x80)).c_str(), function_name);
        log_message(str(format("Return code=%d") % (int)return_code()).c_str(),
                              function_name);
        log_message(str(format("Payload length=%d") % payload_length()).c_str(),
                              function_name);
        log_separator();
#endif
    }
}

uint8_t RemoteObject::process_command(uint8_t cmd) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  const char* function_name = "process_command()";
  log_message(str(format("command=0x%0X (%d)") % cmd % cmd).c_str(),
              function_name);
#endif
  switch(cmd) {
#if defined(AVR) || defined(__SAM3X8E__) // Commands that only the Arduino handles
    case CMD_GET_PROTOCOL_NAME:
      if (payload_length() == 0) {
        serialize(protocol_name(), strlen(protocol_name()));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_PROTOCOL_VERSION:
      if (payload_length() == 0) {
        serialize(protocol_version(), strlen(protocol_version()));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_DEVICE_NAME:
      if (payload_length() == 0) {
        serialize(name(), strlen(name()));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_MANUFACTURER:
      if (payload_length() == 0) {
        serialize(manufacturer(), strlen(manufacturer()));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_SOFTWARE_VERSION:
      if (payload_length() == 0) {
        serialize(software_version(), strlen(software_version()));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_HARDWARE_VERSION:
      if (payload_length() == 0) {
        serialize(hardware_version(), strlen(hardware_version()));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_URL:
      if (payload_length() == 0) {
        serialize(url(), strlen(url()));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_MCU_TYPE:
      if (payload_length() == 0) {
        serialize(MCU_TYPE_, strlen(MCU_TYPE_));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_PIN_MODE:
      if (payload_length() == 2) {
        uint8_t pin = read<uint8_t>();
        uint8_t mode = read<uint8_t>();
        pinMode(pin, mode);
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_DIGITAL_WRITE:
      if (payload_length() == 2) {
        uint8_t pin = read<uint8_t>();
        uint8_t value = read<uint8_t>();
        digitalWrite(pin, value);
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_DIGITAL_READ:
      if (payload_length() == 1) {
        uint8_t pin = read<uint8_t>();
        uint8_t value = digitalRead(pin);
        serialize(&value,sizeof(value));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_ANALOG_WRITE:
      if (payload_length() == 2) {
        uint8_t pin =  read<uint8_t>();
        uint16_t value = read<uint16_t>();
        analogWrite(pin, value);
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_ANALOG_READ:
      uint8_t n_channels;
      uint16_t n_samples;
      return_code_ = RETURN_BAD_PACKET_SIZE;
      if (payload_length() >= 2) {
        n_channels = read<uint8_t>();
        uint8_t pins[n_channels];
        if ( (payload_length() == 2 && n_channels == 1) ||
             (payload_length() == (1+n_channels)*sizeof(uint8_t) +
                 sizeof(uint16_t)) ) {
          for (uint8_t i = 0; i < n_channels; i++) {
            pins[i] = read<uint8_t>();
          }
          if (payload_length() == 2) {
            n_samples = 1;
          } else {
            n_samples = read<uint16_t>();
          }
          if (n_samples > (MAX_PAYLOAD_LENGTH)/sizeof(uint16_t)) {
            return_code_ = RETURN_MAX_PAYLOAD_EXCEEDED;
          } else {
            return_code_ = RETURN_OK;
            // perform analog reads
            AdvancedADC.setBuffer((uint16_t*)payload(), n_samples);
            AdvancedADC.setChannels(pins, n_channels);
            AdvancedADC.begin();
            while(!AdvancedADC.finished());
            // update the number of bytes written
            bytes_written(n_samples*sizeof(uint16_t));
          }
        }
      }
      break;
    case CMD_PERSISTENT_WRITE:
      if (payload_length() == 3) {
        uint16_t address = read<uint16_t>();
        uint8_t value = read<uint8_t>();
        persistent_write(address, value);
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_PERSISTENT_READ:
      if (payload_length() == 2) {
        uint16_t address = read<uint16_t>();
        uint8_t value = persistent_read(address);
        serialize(&value,sizeof(value));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_ONEWIRE_GET_ADDRESS:
      if (payload_length() == 2) {
        uint8_t pin = read<uint8_t>();
        uint8_t index = read<uint8_t>();
        uint8_t addr[8];
        uint8_t ret;
        OneWire ow(pin);
        ow.reset_search();
        for (uint8_t i = 0; i <= index; i++) {
          ret = ow.search(addr);
        }
        if (ret) {
          serialize(addr, 8*sizeof(uint8_t));
          return_code_ = RETURN_OK;
        } else { // No more addresses
          return_code_ = RETURN_BAD_INDEX;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_ONEWIRE_READ:
      if (payload_length() == 11) {
        uint8_t pin = read<uint8_t>();
        uint8_t addr[8];
        for (uint8_t i = 0; i < 8; i++) {
          addr[i] = read<uint8_t>();
        }
        uint8_t command = read<uint8_t>();
        uint8_t n_bytes = read<uint8_t>();
        OneWire ow(pin);
        ow.reset();
        ow.select(addr);
        ow.write(command);
        for (uint8_t i = 0; i < n_bytes; i++) {
          uint8_t data = ow.read();
          serialize(&data, sizeof(uint8_t));
        }
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_ONEWIRE_WRITE:
      if (payload_length() == 11) {
        uint8_t pin = read<uint8_t>();
        uint8_t addr[8];
        for (uint8_t i = 0; i < 8; i++) {
          addr[i] = read<uint8_t>();
        }
        uint8_t value = read<uint8_t>();
        uint8_t power = read<uint8_t>();
        OneWire ow(pin);
        ow.reset();
        ow.select(addr);
        ow.write(value,power);
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_I2C_READ:
      if (payload_length() == 2) {
        uint8_t n_bytes_read = 0;
        uint8_t address = read<uint8_t>();
        uint8_t n_bytes_to_read = read<uint8_t>();
        Wire.requestFrom(address, n_bytes_to_read);
        while (Wire.available()) {
          uint8_t data = Wire.read();
          serialize(&data, sizeof(uint8_t));
          n_bytes_read++;
        }
        if (n_bytes_read == n_bytes_to_read) {
          return_code_ = RETURN_OK;
        } else {
          return_code_ = RETURN_GENERAL_ERROR;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_I2C_WRITE:
      if (payload_length() >= 1) {
        uint8_t address = read<uint8_t>();
        Wire.beginTransmission(address);
        for (uint8_t i = 0; i < payload_length()-1; i++) {
          Wire.write(read<uint8_t>());
        }
        Wire.endTransmission();
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SPI_SET_BIT_ORDER:
      if (payload_length() == 1) {
        uint8_t order = read<uint8_t>();
#ifdef __SAM3X8E__
        SPI.setBitOrder((BitOrder)order);
#else
        SPI.setBitOrder(order);
#endif
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SPI_SET_CLOCK_DIVIDER:
      if (payload_length() == 1) {
        uint8_t divider = read<uint8_t>();
        SPI.setClockDivider(divider);
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SPI_SET_DATA_MODE:
      if (payload_length() == 1) {
        uint8_t mode = read<uint8_t>();
        SPI.setDataMode(mode);
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SPI_TRANSFER:
      if (payload_length() == 1) {
        uint8_t value = read<uint8_t>();
        uint8_t data = SPI.transfer(value);
        serialize(&data, sizeof(data));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_DEBUG_BUFFER:
      if (payload_length() == 0) {
        serialize(debug_buffer_, debug_buffer_length_);
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#endif
#ifdef AVR // only on Arduino Mega 2560
    case CMD_GET_SAMPLING_RATE:
      if (payload_length() == 0) {
        float sampling_rate = AdvancedADC.samplingRate();
        serialize(&sampling_rate, sizeof(sampling_rate));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_SAMPLING_RATE:
      if (payload_length() == sizeof(float)) {
        float sampling_rate = read_float();
        AdvancedADC.setSamplingRate(sampling_rate);
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_ADC_PRESCALER:
      if (payload_length() == 0) {
        uint16_t prescaler = AdvancedADC.prescaler();
        serialize(&prescaler, sizeof(prescaler));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_SET_ADC_PRESCALER:
      if (payload_length() == sizeof(uint16_t)) {
        uint16_t prescaler = read_uint16();
        if (prescaler >=2 && prescaler <= 7) {
          AdvancedADC.setPrescaler((uint8_t)prescaler);
          AdvancedADC.setAutoTrigger(false);
          return_code_ = RETURN_OK;
        } else {
          return_code_ = RETURN_BAD_INDEX;
        }
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_GET_AREF:
      if (payload_length() == 0) {
        serialize(&aref_, sizeof(aref_));
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_I2C_SCAN:
      if (payload_length() == 0) {
        i2c_scan(true);
        return_code_ = RETURN_OK;
      } else {
        return_code_ = RETURN_BAD_PACKET_SIZE;
      }
      break;
#endif
  }
  send_reply(return_code_);
  return return_code_;
}

void RemoteObject::process_serial_input(uint8_t byte) {
    /*
     * Process the next available _byte_ in the serial receive buffer.
     */
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
    const char* function_name = "process_serial_input()";
#endif
    /* Since the packet grammar defined in `Remoteobject.h` reserves certain
     * byte values as special markers _(e.g., `start_flag`, `end_flag`), we
     * provide a mechanism to [escape][1] a reserved byte value, such that it
     * can be included in a packet's payload without disrupting the processing
     * of the packet.  To do this, we allow reserved byte values to be
     * prepended by an [escape character][1] with the value `CONTROL_ESCAPE`.
     *
     * [1]: http://en.wikipedia.org/wiki/Escape_character
     */
    if (byte == CONTROL_ESCAPE) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
        log_message(str(format("(0x%0X) Escape") % (int)byte).c_str(),
                              function_name);
#endif
        /* Update our state to indicate that we are currently processing an
         * [escape character][1] sequence and will need to return to normal
         * processing for the next byte.
         *
         * [1]: http://en.wikipedia.org/wiki/Escape_character
         */
        un_escaping_ = true;
        return;
    } else if (un_escaping_) {
        /* TODO: What does `ESCAPE_XOR` do? */
        byte ^= ESCAPE_XOR;
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
        log_message(str(format("(0x%0X) Un-escaping") % (int)byte).c_str(),
                              function_name);
#endif
    }

    if (byte == FRAME_BOUNDARY && !un_escaping_) {
        /* The current byte marks a frame-boundary, since it matches the
         * `start_flag` or `end_flag` of the packet grammar defined in
         * `Remoteobject.h`.
         *
         * __NB__ If `un_escaping_` is `true`, it would indicate that the
         * previous byte was an [escape character][1] and that the current byte
         * has been XOR'ed with `ESCAPE_XOR` to ensure it does not conflict
         * with one of the reserved byte values.  Therefore, if the previous
         * character was an escape character and the current byte _(which has
         * been XOR'ed with `ESCAPE_XOR`)_ is equal to the `FRAME_BOUNDARY`, we
         * should not actually treat it as a `FRAME_BOUNDARY`.
         *
         * [1]: http://en.wikipedia.org/wiki/Escape_character
         */
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
        log_separator();
        log_message(str(format("(0x%0X) Frame Boundary") % (int)byte).c_str(),
                              function_name);
#endif
        if (bytes_received_ > 0) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
            /* Since our current processing state indicates that we have
             * already received some bytes, we are not expecting a
             * `FRAME_BOUNDARY`, so log the packet as invalid.
             */
            log_message(str(format("(0x%0X) Invalid packet") %
                (int)byte).c_str(), function_name);
#endif
        }
        bytes_received_ = 0;
    } else {
        if (bytes_received_ == 0) { // command byte
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
            log_message(str(format("(0x%0X) Command byte (%d)") % (int)byte %
                                  (int)byte).c_str(), function_name);
#endif
            packet_cmd_ = byte;
            if (crc_enabled_) {
                rx_crc_ = 0xFFFF; // reset the crc
            }
        } else if (bytes_received_ == 1) { // payload length
            if (byte & 0x80) {
                header_length_ = 3;
                payload_length_ = (byte & 0x7F) << 8;
            } else {
                header_length_ = 2;
                payload_length_ = byte;
            }
        // payload length (byte 2)
        } else if (bytes_received_ == 2 && header_length_ == 3) {
            payload_length_ += byte;
        } else if (bytes_received_ - header_length_ < payload_length_) {
            // payload
            // TODO: check that MAX_PAYLOAD_LENGTH isn't exceeded
            payload_[bytes_received_ - header_length_] = byte;
        } else if (bytes_received_ - header_length_ < payload_length_ + 2) {
            // crc
        } else {
          // TODO: error
        }
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
        if (bytes_received_ == header_length_) {
            /* We've received all header bytes, so we can report the
             * payload-length _(which was included in the packet header)_. */
            log_message(str(format("Payload length=%d") %
                        payload_length_).c_str(), function_name);
        }
#endif
        if (crc_enabled_) {
            /* CRC is enabled, so update the CRC-checksum with the current
             * byte. */
            rx_crc_ = update_crc(rx_crc_, byte);
        }
        bytes_received_++;
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
        if (byte >= 0x20 && byte <= 0x7E) {
            /* The byte value is within the [ASCII][1] character range, so
             * display the corresponding character.
             *
             * [1]: http://www.asciitable.com */
            log_message(str(format("(0x%0X) %d bytes received (\'%c\')") %
                        (int)byte % bytes_received_ % byte).c_str(),
                        function_name);
        } else {
            log_message(str(format("(0x%0X) %d bytes received") % (int)byte %
                                  bytes_received_).c_str(), function_name);
        }
#endif
        if (bytes_received_ ==
            payload_length_ + header_length_ + 2 * crc_enabled_) {
            /* We have read the number of bytes that make up a packet,
             * according to the grammar defined in `RemoteObject.h`, so process
             * the packet contents.
             *
             * __NB__ Recall that the length of a packet varies according to
             * whether or not a CRC-value is provided.  This is taken into
             * account here by including the `2 * crc_enabled_` term in the
             * condition above.  In the case where CRC-checking is not enabled,
             * the term will evaluate to 0.
             */
            bytes_received_ = 0;
            bytes_read_ = 0;
            bytes_written_ = 0;
            if (crc_enabled_) {
                /* CRC checking is enabled, so verify that the CRC matches the
                 * CRC value sent in the packet. */
                if (rx_crc_ == 0) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
                    log_message("End of Packet. CRC OK.", function_name);
#endif
                    process_packet();
                } else {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
                    log_message("End of Packet. CRC Error.", function_name);
#endif
                }
            } else {
                /* CRC checking is not enabled, so just proceed with processing
                 * the packet contents. */
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
                log_message("End of Packet", function_name);
#endif
                process_packet();
            }
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
            log_separator();
#endif
            // If we're not expecting something else, stop waiting
            if (waiting_for_reply_to_ &&
                packet_cmd_ == (waiting_for_reply_to_ ^ 0x80)) {
                waiting_for_reply_to_ = 0;
            }
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
            else {
                log_message("Not the expected reply, keep waiting.",
                            function_name);
            }
#endif
        }
    }
    if (un_escaping_) {
        /* If we were handling the escaping of a byte, reset the escaping state
         * to resume normal packet processing. */
        un_escaping_ = false;
    }
}

void RemoteObject::listen() {
    while (Serial.available() > 0) {
        process_serial_input(Serial.read());
    }
}

#if defined(AVR) || defined(__SAM3X8E__)
////////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the Arduino
//
////////////////////////////////////////////////////////////////////////////////

void /* DEVICE */ RemoteObject::begin() {
  uint32_t baud_rate;
  for (uint8_t i = 0; i < 4; i++) {
    ((uint8_t*)&baud_rate)[i] = persistent_read(PERSISTENT_BAUD_RATE_ADDRESS
                                                + i);
  }
  // if the baud rate hasn't been set (default is that all bits are high)
  if (baud_rate == 0xFFFFFFFF) {
    baud_rate = 115200;
    // write the baud_rate to eeprom
    for (uint8_t i = 0; i < 4; i++) {
      persistent_write(PERSISTENT_BAUD_RATE_ADDRESS + i,
        ((uint8_t*)&baud_rate)[i]);
    }
  }
  Serial.begin(baud_rate);
  Wire.begin();
  SPI.begin();

  aref_ = AdvancedADC.readVcc();
  Serial.print("Analog Reference=");
  Serial.print(aref_);
  Serial.println(" V");
}

void /* DEVICE */ RemoteObject::i2c_scan(bool serialize_to_payload) {
  for (uint8_t i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      if (serialize_to_payload) {
        serialize(&i, sizeof(i));
      }
      Serial.print("Found i2c address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println (")");
      delay(1);  // maybe unneeded?
    }
  }
}

void /* DEVICE */ RemoteObject::i2c_write(const uint8_t address,
                                          const uint8_t data) {
    Wire.beginTransmission(address);
    Wire.write(data);
    Wire.endTransmission();
}

void /* DEVICE */ RemoteObject::i2c_write(const uint8_t address,
                                          const uint8_t* data,
                                          const uint8_t n_bytes) {
    Wire.beginTransmission(address);
    for (uint8_t i = 0; i < n_bytes; i++) {
        Wire.write(data[i]);
    }
    Wire.endTransmission();
}

uint8_t /* DEVICE */ RemoteObject::i2c_read(const uint8_t address,
                                            uint8_t* data,
                                            const uint8_t n_bytes_to_read) {
    uint8_t n_bytes_read = 0;
    Wire.requestFrom(address, n_bytes_to_read);
    while (Wire.available()) {
        data[n_bytes_read++] = Wire.read();
    }
    return n_bytes_read;
}

uint8_t /* HOST */ RemoteObject::persistent_read(uint16_t address) {
    return EEPROM.read(address);
}

void /* HOST */ RemoteObject::persistent_write(uint16_t address,
                                               uint8_t value) {
    EEPROM.write(address, value);
}

#else
////////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the PC
//
////////////////////////////////////////////////////////////////////////////////

uint8_t /* HOST */ RemoteObject::connect(const char* port,
                                         uint32_t baud_rate) {
  const char* function_name = "connect()";
  int return_code = Serial.begin(port, baud_rate);

  // wait up to 10 s for the Arduino to send something on the
  // serial port so that we know it's ready
  if (return_code == 0) {
    boost::posix_time::ptime t =
      boost::posix_time::microsec_clock::universal_time();
    while (Serial.available()==false && \
      (boost::posix_time::microsec_clock::universal_time()-t)
      .total_seconds()<10) {
    }
    // flush the serial buffer to clear startup message
    while (Serial.available()) {
      // This delay needs to be >=500 ms for R2 and R3 boards. Use 1000 ms to
      // be safe.
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
      Serial.flush();
    }
  }
  log_message(str(format("Serial.begin(%s, %d)=%d") % port % baud_rate %
    return_code).c_str(), function_name);

  if (return_code == 0) {
    // verify that the device name and hardware version are correct
    std::string remote_name;
    std::string remote_hardware_version;
    try {
      remote_name = name();
      remote_hardware_version = hardware_version();
    } catch(...) {
      // close the Serial port if we can't get a name/version
      Serial.end();
      throw;
    }
    log_message(str(format("name()=\"%s\", hardware_version()=\"%s\"") %
      remote_name % remote_hardware_version).c_str(), function_name);
    if (remote_name == host_name()) {
      return RETURN_OK;
    } else {
      Serial.end();
      throw runtime_error(str(format("Remote device is not a %s.") %
        host_name()).c_str());
    }
  }
  throw runtime_error(str(format("Could not connect to port %s.") %
    port).c_str());
}

void /* HOST */ RemoteObject::set_debug(const bool debug) {
  debug_ = debug;
}

string /* HOST */ RemoteObject::protocol_name() {
  const char* function_name = "protocol_name()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_GET_PROTOCOL_NAME) == RETURN_OK) {
    string protocol_name = read_string();
    log_message(str(format("protocol_name=%s") % protocol_name).c_str(),
      function_name);
    return protocol_name;
  }
  return "";
}

string /* HOST */ RemoteObject::protocol_version() {
  const char* function_name = "protocol_version()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_GET_PROTOCOL_VERSION) == RETURN_OK) {
    string protocol_version = read_string();
    log_message(str(format("protocol_version=%s") % protocol_version).c_str(),
      function_name);
    return protocol_version;
  }
  return "";
}

string /* HOST */ RemoteObject::name() {
  const char* function_name = "name()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_GET_DEVICE_NAME) == RETURN_OK) {
    string name = read_string();
    log_message(str(format("name=%s") % name).c_str(), function_name);
    return name;
  }
  return "";
}

string /* HOST */ RemoteObject::manufacturer() {
  const char* function_name = "manufacturer()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_GET_MANUFACTURER) == RETURN_OK) {
    string manufacturer = read_string();
    log_message(str(format("manufacturer=%s") % manufacturer).c_str(),
      function_name);
    return manufacturer;
  }
  return "";
}

string /* HOST */ RemoteObject::software_version() {
  const char* function_name = "software_version()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_GET_SOFTWARE_VERSION) == RETURN_OK) {
    string software_version = read_string();
    log_message(str(format("software_version=%s") % software_version).c_str(),
      function_name);
    return software_version;
  }
  return "";
}

string /* HOST */ RemoteObject::hardware_version() {
  const char* function_name = "hardware_version()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_GET_HARDWARE_VERSION) == RETURN_OK) {
    string hardware_version = read_string();
    log_message(str(format("hardware_version=%s") % hardware_version).c_str(),
      function_name);
    return hardware_version;
  }
  return "";
}

string /* HOST */ RemoteObject::url() {
  const char* function_name = "url()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_GET_URL) == RETURN_OK) {
    string url = read_string();
    log_message(str(format("url=%s") % url).c_str(), function_name);
    return url;
  }
  return "";
}

string /* HOST */ RemoteObject::mcu_type() {
  const char* function_name = "mcu_type()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_GET_MCU_TYPE) == RETURN_OK) {
    string mcu_type = read_string();
    log_message(str(format("mcu_type=%s") % mcu_type).c_str(),
      function_name);
    return mcu_type;
  }
  return "";
}

void /* HOST */ RemoteObject::set_pin_mode(uint8_t pin, bool mode) {
  const char* function_name = "set_pin_mode()";
  log_separator();
  log_message("send command", function_name);
  serialize(&pin,sizeof(pin));
  uint8_t data = mode;
  serialize(&data,sizeof(data));
  if (send_command(CMD_SET_PIN_MODE) == RETURN_OK) {
    log_message(str(format("pin %d mode=%d") % pin % mode).c_str(),
      function_name);
  }
}

uint8_t /* HOST */ RemoteObject::digital_read(uint8_t pin) {
  const char* function_name = "digital_read()";
  log_separator();
  log_message("send command", function_name);
  serialize(&pin,sizeof(pin));
  if (send_command(CMD_DIGITAL_READ) == RETURN_OK) {
    uint8_t value = read<uint8_t>();
    log_message(str(format("pin %d value=%d") % pin % value).c_str(),
      function_name);
    return value;
  }
  return 0;
}

void /* HOST */ RemoteObject::digital_write(uint8_t pin, bool value) {
  const char* function_name = "digital_write()";
  log_separator();
  log_message("send command", function_name);
  serialize(&pin,sizeof(pin));
  uint8_t data = value;
  serialize(&data,sizeof(data));
  if (send_command(CMD_DIGITAL_WRITE) == RETURN_OK) {
    log_message(str(format("pin %d value=%d") % pin % value).c_str(),
      function_name);
  }
}

uint16_t /* HOST */ RemoteObject::analog_read(uint8_t pin) {
  const char* function_name = "analog_read()";
  log_separator();
  log_message("send command", function_name);
  uint8_t n_channels = 1;
  serialize(&n_channels,sizeof(n_channels));
  serialize(&pin,sizeof(pin));
  if (send_command(CMD_ANALOG_READ) == RETURN_OK) {
    uint16_t value = read<uint16_t>();
    log_message(str(format("pin %d value=%d") % pin % value).c_str(),
      function_name);
    return value;
  }
  return 0;
}

std::vector<uint16_t> /* HOST */ RemoteObject::analog_reads(
    std::vector<uint8_t> pins, uint16_t n_samples) {
  const char* function_name = "analog_reads()";
  log_separator();
  log_message("send command", function_name);
  uint8_t n_channels = pins.size();
  serialize(&n_channels, sizeof(n_channels));
  serialize(&pins[0], n_channels*sizeof(uint8_t));
  serialize(&n_samples, sizeof(n_samples));
  if (send_command(CMD_ANALOG_READ) == RETURN_OK) {
    if (payload_length()==n_samples*sizeof(uint16_t)) {
      std::vector<uint16_t> buffer(n_samples);
      for (uint16_t i=0; i<n_samples; i++) {
        buffer[i] = read<uint16_t>();
      }
      return buffer;
    } else {
      return_code_ = RETURN_BAD_PACKET_SIZE;
    }
  }
  return std::vector<uint16_t>();
}

void /* HOST */ RemoteObject::analog_write(uint8_t pin, uint16_t value) {
  const char* function_name = "analog_write()";
  log_separator();
  log_message("send command", function_name);
  serialize(&pin,sizeof(pin));
  serialize(&value,sizeof(value));
  if (send_command(CMD_DIGITAL_WRITE) == RETURN_OK) {
    log_message(str(format("pin %d value=%d") % pin % value).c_str(),
      function_name);
  }
}

uint8_t /* HOST */ RemoteObject::persistent_read(uint16_t address) {
  const char* function_name = "persistent_read()";
  log_separator();
  log_message("send command", function_name);
  serialize(&address,sizeof(address));
  if (send_command(CMD_PERSISTENT_READ) == RETURN_OK) {
    uint8_t value = read<uint8_t>();
    log_message(str(format("address %d value=%d") % address % value).c_str(),
      function_name);
    return value;
  }
  return 0;
}

void /* HOST */ RemoteObject::persistent_write(uint16_t address,
                                               uint8_t value) {
  const char* function_name = "persistent_write()";
  log_separator();
  log_message("send command", function_name);
  serialize(&address,sizeof(address));
  serialize(&value,sizeof(value));
  if (send_command(CMD_PERSISTENT_WRITE) == RETURN_OK) {
    log_message(str(format("address %d value=%d") % address % value).c_str(),
      function_name);
  }
}

std::vector<uint8_t> /* HOST */ RemoteObject::onewire_address(uint8_t pin,
                                                              uint8_t index) {
  const char* function_name = "onewire_address()";
  log_separator();
  log_message("send command", function_name);
  serialize(&pin, sizeof(pin));
  serialize(&index, sizeof(index));
  if (send_command(CMD_ONEWIRE_GET_ADDRESS) == RETURN_OK) {
    log_message(str(format("pin %d, index=%d") % pin % index).c_str(),
      function_name);
    std::vector<uint8_t> address;
    for (int i = 0; i < payload_length(); i++) {
      address.push_back(read<uint8_t>());
    }
    return address;
  }
  return std::vector<uint8_t>();
}

std::vector<uint8_t> /* HOST */ RemoteObject::onewire_read(uint8_t pin,
                                                           std::vector<uint8_t>
                                                           address,
                                                           uint8_t command,
                                                           uint8_t n_bytes) {
  const char* function_name = "onewire_read()";
  log_separator();
  log_message("send command", function_name);
  if (address.size()==8) {
    serialize(&pin,sizeof(pin));
    serialize(&address[0],8*sizeof(uint8_t));
    serialize(&command,sizeof(command));
    serialize(&n_bytes,sizeof(n_bytes));
    if (send_command(CMD_ONEWIRE_READ) == RETURN_OK) {
      std::vector<uint8_t> data;
      for (uint8_t i=0; i<n_bytes; i++) {
        data.push_back(read<uint8_t>());
      }
      log_message(str(format("pin %d, command=%d, n_bytes=%d") % pin % command %
        n_bytes).c_str(), function_name);
      return data;
    }
  }
  return std::vector<uint8_t>();
}

void /* HOST */ RemoteObject::onewire_write(uint8_t pin,
                                            std::vector<uint8_t> address,
                                            uint8_t value, uint8_t power) {
  const char* function_name = "onewire_write()";
  log_separator();
  log_message("send command", function_name);
  if (address.size()==8) {
    serialize(&pin,sizeof(pin));
    serialize(&address[0],8*sizeof(uint8_t));
    serialize(&value,sizeof(value));
    serialize(&power,sizeof(power));
    if (send_command(CMD_ONEWIRE_WRITE) == RETURN_OK) {
      log_message(str(format("pin %d, value=%d, power=%d") % pin % value %
        power).c_str(), function_name);
    }
  }
}

void /* HOST */ RemoteObject::i2c_write(uint8_t address,
                                        std::vector<uint8_t> data) {
  const char* function_name = "i2c_write()";
  log_separator();
  log_message("send command", function_name);
  serialize(&address,sizeof(address));
  serialize(&data[0],data.size()*sizeof(uint8_t));
  if (send_command(CMD_I2C_WRITE) == RETURN_OK) {
    log_message(str(format("address %d") % address).c_str(), function_name);
    for (uint8_t i=0; i<data.size(); i++) {
      log_message(str(format("data[%d]=%d") % i % data[i]).c_str(),
        function_name);
    }
  }
}

std::vector<uint8_t> /* HOST */ RemoteObject::i2c_scan() {
  const char* function_name = "i2c_scan()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_I2C_SCAN) == RETURN_OK) {
    std::vector<uint8_t> received_data;
    for (uint8_t i = 0; i < payload_length(); i++) {
      received_data.push_back(read<uint8_t>());
      log_message(str(format("received_data[%d]=%d") % i %
        received_data[i]).c_str(), function_name);
    }
    return received_data;
  }
  return std::vector<uint8_t>();
}

std::vector<uint8_t> /* HOST */ RemoteObject::i2c_read(uint8_t address,
                                                       uint8_t
                                                       n_bytes_to_read) {
  const char* function_name = "i2c_read()";
  log_separator();
  log_message("send command", function_name);
  serialize(&address,sizeof(address));
  serialize(&n_bytes_to_read,sizeof(n_bytes_to_read));
  if (send_command(CMD_I2C_READ) == RETURN_OK) {
    log_message(str(format("address %d") % address).c_str(), function_name);
    std::vector<uint8_t> received_data;
    for (uint8_t i=0; i<n_bytes_to_read; i++) {
      received_data.push_back(read<uint8_t>());
      log_message(str(format("received_data[%d]=%d") % i %
        received_data[i]).c_str(), function_name);
    }
    return received_data;
  }
  return std::vector<uint8_t>();
}

std::vector<uint8_t> /* HOST */ RemoteObject::i2c_send_command(uint8_t address,
                                                               uint8_t cmd,
                                                               std::vector
                                                               <uint8_t> data,
                                                               uint8_t
                                                               delay_ms) {
  const char* function_name = "i2c_send_command()";
  data.insert(data.begin(), cmd);
  i2c_write(address, data);
  boost::this_thread::sleep(boost::posix_time::milliseconds(delay_ms));
  uint8_t n_bytes = i2c_read(address, 1)[0];
  if (n_bytes == 0) {
    throw runtime_error(str(format("Error sending command 0x%0X (%d). "
      "No return code provided.") % (int)cmd % (int)cmd).c_str());
  }
  std::vector<uint8_t> out = i2c_read(address, n_bytes);
  uint8_t return_code = out.back();
  log_message(str(format("Return code=%d") % (int)return_code).c_str(),
             function_name);
  out.pop_back();
  if (return_code != RETURN_OK) {
    throw runtime_error(str(format("Error sending command 0x%0X (%d). "
      "Return code=%d.") % (int)cmd % (int)cmd % (int)return_code).c_str());
  }
  return out;
}

void /* HOST */ RemoteObject::spi_set_bit_order(bool order) {
    send_set_command(CMD_SPI_SET_BIT_ORDER, "spi_set_bit_order()", order);
}

void /* HOST */ RemoteObject::spi_set_clock_divider(uint8_t divider) {
    send_set_command(CMD_SPI_SET_CLOCK_DIVIDER, "spi_set_clock_divider()",
                     divider);
}

void /* HOST */ RemoteObject::spi_set_data_mode(uint8_t mode) {
    send_set_command(CMD_SPI_SET_DATA_MODE, "spi_set_data_mode()", mode);
}

uint8_t /* HOST */ RemoteObject::spi_transfer(uint8_t value) {
    const char* function_name = "spi_transfer()";
    send_set_command(CMD_SPI_TRANSFER, function_name, value);
    uint8_t data = read<uint8_t>();
    log_message(str(format("sent: %d, received: %d") % value % data).c_str(),
               function_name);
    return data;
}

std::vector<uint8_t> /* HOST */ RemoteObject::debug_buffer() {
  const char* function_name = "debug_buffer()";
  log_separator();
  log_message("send command", function_name);
  if (send_command(CMD_GET_DEBUG_BUFFER) == RETURN_OK) {
    std::vector<uint8_t> buffer;
    for (uint16_t i=0; i<payload_length(); i++) {
      buffer.push_back(read<uint8_t>());
    }
    return buffer;
  }
  return std::vector<uint8_t>();
}

float /* HOST */ RemoteObject::sampling_rate() {
  return send_read_command<float>(CMD_GET_SAMPLING_RATE,
                                  "sampling_rate()");
}

uint8_t /* HOST */ RemoteObject::set_sampling_rate(const float sampling_rate) {
  return send_set_command(CMD_SET_SAMPLING_RATE, "set_sampling_rate()",
                          sampling_rate);
}

uint16_t /* HOST */ RemoteObject::adc_prescaler() {
  return send_read_command<uint16_t>(CMD_GET_ADC_PRESCALER, "prescaler()");
}

uint8_t /* HOST */ RemoteObject::set_adc_prescaler(const uint16_t prescaler) {
  return send_set_command(CMD_SET_ADC_PRESCALER, "set_adc_prescaler()",
                          prescaler);
}

float /* HOST */ RemoteObject::aref() {
  return send_read_command<float>(CMD_GET_AREF, "aref()");
}

#endif
