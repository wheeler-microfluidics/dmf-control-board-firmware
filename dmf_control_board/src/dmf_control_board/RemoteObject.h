/**
RemoteObject:

This class implements a simple communication protocol between a PC and
Arduino over an RS-232 link using HDLC-like framing. It uses a wrapper
for the boost asio library (SimpleSerial) that provides an interface that
closely matches the Serial library on the Arduino. This allows sharing
of the bulk of the code between the PC (Windows, Linux or Mac) and the
Arduino. This implementation was partly inspired by Alvaro Lopes' serpro
project:

  https://github.com/alvieboy/arduino-serpro

Each packet has the following structure:

<pre>
+------------+---------+----------------+---------+------------+----------+
| Start Flag | Command | Payload Length | Payload |    CRC     | End Flag |
|   1 byte   | 1 byte  |    1-2 bytes   | N bytes |  2 bytes   |  1 byte  |
|    0x7E    |         |                |         | (optional) |   0x7E   |
+------------+---------+----------------+---------+------------+----------+
</pre>

Commands are uint8_t and should have the MSB=1 (replies have MSB=0). The
RemoteObject base class reserves the commands 0x80 to 0x9F, while derived
classes should restrict themselves to commands in the range 0xA0 to 0xFF.

The payload length can be one or two bytes.  If the payload is less than 128
bytes, it's length is expressed as a single byte.  If the most-significant
bit is set, the length is expressed as two bytes and can be recovered by
clearing the most significant byte (i.e. PAYLOAD_LENGTH & 0x7FFF).

<b>Examples:</b>
   - payload length of 3, one byte: 0x04
   - payload length of 512, two bytes: 0x82 0x01

Total packet length (not including flags) = Header Length (2-3 bytes)
                                            + Payload Length
                                            (+ 2 if CRC is enabled)

To use this class, you must derive a class based on it and reimplement the
virtual member function process_command().
*/
/*
__________________________________________________________________________

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

__________________________________________________________________________
*/

#ifndef _REMOTE_OBJECT_H
#define	_REMOTE_OBJECT_H

#include <stdint.h>
#include "Deserialize.h"

#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  #include "Logging.h"
  #include "SimpleSerial.h"
  #include <string>
  #include <boost/format.hpp>
#endif


class RemoteObject {
public:
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  static const uint32_t TIMEOUT_MILLISECONDS =   1000; // TODO: this should be configurable
#else
  static const uint8_t I2C_DELAY = 100; // delay between i2c write/reads
                                        // in future, we can avoid this by
                                        // adding interrupts on the
                                        // communication bus
#endif
  // Persistent storage _(e.g., EEPROM)_ addresses.
  static const uint16_t PERSISTENT_PIN_MODE_ADDRESS =       0;
  static const uint16_t PERSISTENT_PIN_STATE_ADDRESS =      7;
  static const uint16_t PERSISTENT_BAUD_RATE_ADDRESS =     15;
  static const uint16_t PERSISTENT_SERIAL_NUMBER_ADDRESS = 19;

  // protocol constants
  static const uint16_t MAX_PAYLOAD_LENGTH =            2000;
  static const uint16_t MAX_DEBUG_BUFFER_LENGTH =       1000;

  // reserved commands
  static const uint8_t CMD_GET_PROTOCOL_NAME =          0x80;
  static const uint8_t CMD_GET_PROTOCOL_VERSION =       0x81;
  static const uint8_t CMD_GET_DEVICE_NAME =            0x82;
  static const uint8_t CMD_GET_MANUFACTURER =           0x83;
  static const uint8_t CMD_GET_HARDWARE_VERSION =       0x84;
  static const uint8_t CMD_GET_SOFTWARE_VERSION =       0x85;
  static const uint8_t CMD_GET_URL =                    0x86;
  static const uint8_t CMD_SET_PIN_MODE =               0x87;
  static const uint8_t CMD_DIGITAL_READ =               0x88;
  static const uint8_t CMD_DIGITAL_WRITE =              0x89;
  static const uint8_t CMD_ANALOG_READ =                0x8A;
  static const uint8_t CMD_ANALOG_WRITE =               0x8B;
  static const uint8_t CMD_PERSISTENT_READ =            0x8C;
  static const uint8_t CMD_PERSISTENT_WRITE =           0x8D;
  static const uint8_t CMD_ONEWIRE_GET_ADDRESS =        0x8E;
  static const uint8_t CMD_ONEWIRE_WRITE =              0x8F;
  static const uint8_t CMD_ONEWIRE_READ =               0x90;
  static const uint8_t CMD_I2C_WRITE =                  0x91;
  static const uint8_t CMD_I2C_READ =                   0x92;
  static const uint8_t CMD_SPI_SET_BIT_ORDER =          0x93;
  static const uint8_t CMD_SPI_SET_CLOCK_DIVIDER =      0x94;
  static const uint8_t CMD_SPI_SET_DATA_MODE =          0x95;
  static const uint8_t CMD_SPI_TRANSFER =               0x96;
  static const uint8_t CMD_GET_DEBUG_BUFFER =           0x97;
  static const uint8_t CMD_GET_MCU_TYPE =               0x98;

  // reserved return codes
  static const uint8_t RETURN_OK =                      0x00;
  static const uint8_t RETURN_GENERAL_ERROR =           0x01;
  static const uint8_t RETURN_UNKNOWN_COMMAND =         0x02;
  static const uint8_t RETURN_TIMEOUT =                 0x03;
  static const uint8_t RETURN_NOT_CONNECTED =           0x04;
  static const uint8_t RETURN_BAD_INDEX =               0x05;
  static const uint8_t RETURN_BAD_PACKET_SIZE =         0x06;
  static const uint8_t RETURN_BAD_CRC =                 0x07;
  static const uint8_t RETURN_BAD_VALUE =               0x08;
  static const uint8_t RETURN_MAX_PAYLOAD_EXCEEDED =    0x09;

  static const char MCU_TYPE_[];

  RemoteObject(bool crc_enabled_
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
                 ,const char* class_name
#endif
                 );
  ~RemoteObject();

  uint8_t return_code() { return return_code_; }
  bool crc_enabled() { return crc_enabled_; }
  void listen();
  void send_interrupt();
  uint16_t bytes_read() { return bytes_read_; }
  bool waiting_for_reply() {
      listen();
      return waiting_for_reply_to_>0;
  }

#if defined(AVR) || defined(__SAM3X8E__)
  virtual void begin();
  void i2c_scan();

  // These methods force the derived class to define functions that
  // return the following attributes:
  virtual const char* protocol_name() = 0;
  virtual const char* protocol_version() = 0;
  virtual const char* name() = 0;
  virtual const char* manufacturer() = 0;
  virtual const char* software_version() = 0;
  virtual const char* hardware_version() = 0;
  virtual const char* url() = 0;
  void i2c_write(const uint8_t address, const uint8_t data);
  void i2c_write(const uint8_t address,
                 const uint8_t* data,
                 const uint8_t n_bytes);
  uint8_t i2c_read(const uint8_t address, uint8_t* data,
                   const uint8_t n_bytes_to_read);
  uint8_t i2c_send_command(uint8_t address,
                           uint8_t cmd,
                           uint8_t* data,
                           uint8_t delay_ms);
  /* The following two `persistent...` methods provide sub-classes a mechanism
   * to customize persistent storage.  For example, the Arduino DUE does not
   * support the `EEPROM` library used by the AVR chips. */
  virtual uint8_t persistent_read(uint16_t address);
  virtual void persistent_write(uint16_t address, uint8_t value);
#else
  virtual std::string host_name() = 0;
  virtual std::string host_software_version() = 0;
  virtual std::string host_url() = 0;
  virtual std::string host_manufacturer() = 0;

    virtual std::string command_label(uint8_t command) const {
        if (command == CMD_GET_PROTOCOL_NAME) {
            return std::string("CMD_GET_PROTOCOL_NAME");
        } else if (command == CMD_GET_PROTOCOL_VERSION) {
            return std::string("CMD_GET_PROTOCOL_VERSION");
        } else if (command == CMD_GET_DEVICE_NAME) {
            return std::string("CMD_GET_DEVICE_NAME");
        } else if (command == CMD_GET_MANUFACTURER) {
            return std::string("CMD_GET_MANUFACTURER");
        } else if (command == CMD_GET_HARDWARE_VERSION) {
            return std::string("CMD_GET_HARDWARE_VERSION");
        } else if (command == CMD_GET_SOFTWARE_VERSION) {
            return std::string("CMD_GET_SOFTWARE_VERSION");
        } else if (command == CMD_GET_URL) {
            return std::string("CMD_GET_URL");
        } else if (command == CMD_SET_PIN_MODE) {
            return std::string("CMD_SET_PIN_MODE");
        } else if (command == CMD_DIGITAL_READ) {
            return std::string("CMD_DIGITAL_READ");
        } else if (command == CMD_DIGITAL_WRITE) {
            return std::string("CMD_DIGITAL_WRITE");
        } else if (command == CMD_ANALOG_READ) {
            return std::string("CMD_ANALOG_READ");
        } else if (command == CMD_ANALOG_WRITE) {
            return std::string("CMD_ANALOG_WRITE");
        } else if (command == CMD_PERSISTENT_READ) {
            return std::string("CMD_PERSISTENT_READ");
        } else if (command == CMD_PERSISTENT_WRITE) {
            return std::string("CMD_PERSISTENT_WRITE");
        } else if (command == CMD_ONEWIRE_GET_ADDRESS) {
            return std::string("CMD_ONEWIRE_GET_ADDRESS");
        } else if (command == CMD_ONEWIRE_WRITE) {
            return std::string("CMD_ONEWIRE_WRITE");
        } else if (command == CMD_ONEWIRE_READ) {
            return std::string("CMD_ONEWIRE_READ");
        } else if (command == CMD_I2C_WRITE) {
            return std::string("CMD_I2C_WRITE");
        } else if (command == CMD_I2C_READ) {
            return std::string("CMD_I2C_READ");
        } else if (command == CMD_SPI_SET_BIT_ORDER) {
            return std::string("CMD_SPI_SET_BIT_ORDER");
        } else if (command == CMD_SPI_SET_CLOCK_DIVIDER) {
            return std::string("CMD_SPI_SET_CLOCK_DIVIDER");
        } else if (command == CMD_SPI_SET_DATA_MODE) {
            return std::string("CMD_SPI_SET_DATA_MODE");
        } else if (command == CMD_SPI_TRANSFER) {
            return std::string("CMD_SPI_TRANSFER");
        } else if (command == CMD_GET_DEBUG_BUFFER) {
            return std::string("CMD_GET_DEBUG_BUFFER");
        } else if (command == CMD_GET_MCU_TYPE) {
            return std::string("CMD_GET_MCU_TYPE");
        } else {
            throw std::runtime_error("Invalid command.");
        }
    }
  /////////////////////////////////////////////////////////////////////////
  //
  // Remote accessors
  //
  /////////////////////////////////////////////////////////////////////////

  /**\brief Get the remote device's protocol name.*/
  std::string protocol_name();
  /**\brief Get the remote device's protocol version.*/
  std::string protocol_version();
  /**\brief Get the remote device's name.*/
  std::string name();
  /**\brief Get the remote device's manufacturer.*/
  std::string manufacturer();
  /**\brief Get the remote device's software version.*/
  std::string software_version();
  /**\brief Get the remote device's hardware version.*/
  std::string hardware_version();
  /**\brief Get the remote device's url.*/
  std::string url();
  /**\brief Get the mcu type of the remote device.*/
  std::string mcu_type();
  std::vector<uint8_t> debug_buffer();
  void set_pin_mode(uint8_t pin, bool mode);
  uint8_t digital_read(uint8_t pin);
  void digital_write(uint8_t pin, bool value);
  uint16_t analog_read(uint8_t pin);
  std::vector<uint16_t> analog_reads(uint8_t pin,
                                     uint16_t n_samples);
  void analog_write(uint8_t pin, uint16_t value);
  uint8_t persistent_read(uint16_t address);
  void persistent_write(uint16_t address, uint8_t value);
  std::vector<uint8_t> onewire_address(uint8_t pin, uint8_t index);
  std::vector<uint8_t> onewire_read(uint8_t pin, std::vector<uint8_t> address,
                                    uint8_t command, uint8_t n_bytes);
  void onewire_write(uint8_t pin, std::vector<uint8_t> address,
                     uint8_t value, uint8_t power);
  void i2c_write(uint8_t address, std::vector<uint8_t> data);
  std::vector<uint8_t> i2c_read(uint8_t address,
                                uint8_t n_bytes_to_read);
  std::vector<uint8_t> i2c_send_command(uint8_t address,
                                        uint8_t cmd,
                                        std::vector<uint8_t> data,
                                        uint8_t delay_ms);

  /**Set the order of the bits shifted out of and into the SPI bus, either
  LSBFIRST (least-significant bit first) or MSBFIRST (most-significant bit
  first).
  \param order either LSBFIRST or MSBFIRST
  \returns None
  \sa spi_set_data_mode()
  */
  void spi_set_bit_order(bool order);

  /**Set the SPI clock divider relative to the system clock. The dividers
  available are 2, 4, 8, 16, 32, 64, or 128. The default setting is
  SPI_CLOCK_DIV4, which sets the SPI clock to one-quarter the frequency of
  the system clock.
  \param divider
    SPI_CLOCK_DIV2,
    SPI_CLOCK_DIV4,
    SPI_CLOCK_DIV8,
    SPI_CLOCK_DIV16,
    SPI_CLOCK_DIV32,
    SPI_CLOCK_DIV64, or
    SPI_CLOCK_DIV128
  \returns None
  */
  void spi_set_clock_divider(uint8_t divider);

  /**Set the SPI data mode: that is, clock polarity and phase. See
  <a href="http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus">
  the Wikipedia article on SPI</a> for details.
  \param mode SPI_MODE0, SPI_MODE1, SPI_MODE2, or SPI_MODE3
  \returns None
  \sa spi_set_bit_order()
  */
  void spi_set_data_mode(uint8_t mode);

  /**Transfers one byte over the SPI bus, both sending and receiving.
  \param value the byte to send out over the bus
  \returns the byte read from the bus
  */
  uint8_t spi_transfer(uint8_t value);

  void set_debug(const bool debug);
  bool connected() { return Serial.isOpen(); }
  uint8_t connect(const char* port, uint32_t baud_rate);
  uint8_t disconnect() { Serial.end(); return RETURN_OK; }
  void flush() { Serial.flush(); }

  template <typename Output>
  Output send_read_command(uint8_t command, const char* function_name) {
      log_separator();
      log_message("send command", function_name);
      if (send_command(command) == RETURN_OK) {
          log_message(command_label(command).c_str(), function_name);
          if (payload_length() == sizeof(Output)) {
              Output value = read<Output>();
              log_message(boost::str(boost::format(command_label(command) +
                                                  "=" +
                                                  type_format<Output>()) %
                                    value).c_str(), function_name);
              return value;
          } else {
              log_message((command_label(command) +
                          ", Bad packet size").c_str(), function_name);
              throw std::runtime_error("Bad packet size.");
          }
      }
      throw std::runtime_error("Error processing command.");
  }

  template <typename T>
  uint8_t send_set_command(uint8_t command, const char* function_name,
                           T value) {
      log_separator();
      log_message("send command", function_name);
      serialize(&value, sizeof(value));
      if (send_command(command) == RETURN_OK) {
          log_message(command_label(command).c_str(), function_name);
          log_message("  --> set successfully", function_name);
      }
      return return_code();
  }

#endif

protected:
  /**Process a received command. Derived classes should reimplement this
  function to handle additional commands and they should also make sure to
  call the base class class method.
  \param cmd command code
  \returns RETURN_OK if successfull
  */
  virtual uint8_t process_command(uint8_t cmd) = 0;
  uint16_t payload_length() { return payload_length_; }

  // WARNING: The following two functions should only be used if you really
  // know what you are doing!  In most cases you can just use serialize().
  uint8_t* payload() { return payload_; } // pointer to the payload buffer
  void bytes_written(uint16_t bytes) { bytes_written_+=bytes; }

  template<typename T>
    void serialize(T data,uint16_t size) {
      serialize((const uint8_t*)data,size); }
  void serialize(const uint8_t* u, const uint16_t size);
  void send_reply(const uint8_t return_code);

  template<typename T> void read_array(T* array, const uint16_t size) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
    log_message("","read_array()");
#endif
    bytes_read_ += size;
    memcpy(array,payload_+bytes_read_-size,size);
  }
  const char* read_string();

  template <typename T>
  T read() {
    T result;
    uint32_t size = deserialize(payload_ + bytes_read_, result);
    bytes_read_ += size;
#if !( defined(AVR) || defined(__SAM3X8E__) )
    std::string function_name = "read<" + type_label<T>() + ">";
    std::string format_str = "=" + type_format<T>() + ", bytes_read_=%d";
    log_message(boost::str(boost::format(format_str) % result %
               bytes_read_).c_str(), function_name.c_str());
#endif
    return result;
  }
  uint16_t read_uint16() { return read<uint16_t>(); }
  int16_t read_int16() { return read<int16_t>(); }
  uint8_t read_uint8() { return read<uint8_t>(); }
  int8_t read_int8() { return read<int8_t>(); }
  float read_float() { return read<float>(); }
  uint8_t wait_for_reply();

  /**
  Send a command packet to the Arduino. Prior to calling this function, the
  caller should serialize any data that needs to be included in the payload.
  \returns RETURN_OK if successfull.
  \sa serialize()
  */
  uint8_t send_command(const uint8_t cmd);
  void send_non_blocking_command(const uint8_t cmd);
  uint8_t validate_reply(const uint8_t cmd);

#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  inline void log_message(const char* msg,
                          const char* function_name,
                          uint8_t level=5) {
    if(debug_) {
      Logging::log_message(level,msg,class_name_.c_str(), function_name);
    }
  }
  inline void log_error(const char* msg, const char* function_name) {
    if(debug_) {
            Logging::log_error(msg,class_name_.c_str(), function_name);
    }
  }
  inline void log_separator() { if(debug_) { Logging::log_separator(); }}
#endif
  char debug_buffer_[MAX_DEBUG_BUFFER_LENGTH];
  uint16_t debug_buffer_length_;
  uint8_t return_code_; // return code
  uint8_t packet_cmd_; // command

private:
  static const uint8_t FRAME_BOUNDARY =           0x7E;
  static const uint8_t CONTROL_ESCAPE =           0x7D;
  static const uint8_t ESCAPE_XOR =               0x20;

  void send_preamble(const uint8_t cmd);
  void send_payload();
  void send_byte(uint8_t b);
  uint16_t update_crc(uint16_t crc, uint8_t data);
  void process_serial_input(const uint8_t byte);
  void process_packet();

  uint8_t payload_[MAX_PAYLOAD_LENGTH+2]; // payload (+1 or 2 bytes for payload
                                          // length)
  uint16_t payload_length_; // length of the payload
  uint8_t header_length_; // length of the packet header (2 if payload is
                          // <128 bytes, 3 otherwise)
  uint16_t bytes_received_; // bytes received so far in packet
  uint16_t bytes_read_; // bytes that have been read (by read methods)
  uint16_t bytes_written_; // bytes that have been written (by serialize method)
  bool un_escaping_; // flag that the last byte was an escape
  bool is_reply; // flag that the command we are processing is a reply
  uint8_t waiting_for_reply_to_; // command that we are expecting a reply to
  bool crc_enabled_;
  uint16_t tx_crc_;
  uint16_t rx_crc_;
  bool debug_;
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  SimpleSerial Serial;
  std::string class_name_;
  boost::posix_time::ptime time_cmd_sent_;
#endif
};

#endif	// _REMOTE_OBJECT_H
