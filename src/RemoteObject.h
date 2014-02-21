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
virtual member function ProcessCommand().
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

#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  #include "logging.h"
  #include "SimpleSerial.h"
  #include <string>
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
  static const uint16_t PERSISTENT_PIN_MODE_ADDRESS =      0;
  static const uint16_t PERSISTENT_PIN_STATE_ADDRESS =     7;
  static const uint16_t PERSISTENT_BAUD_RATE_ADDRESS =    15;

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

  RemoteObject(bool crc_enabled_
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
                 ,const char* class_name
#endif
                 );
  ~RemoteObject();

  uint8_t return_code() { return return_code_; }
  bool crc_enabled() { return crc_enabled_; }
  void Listen();
  void SendInterrupt();
  uint16_t bytes_read() { return bytes_read_; }
  bool waiting_for_reply() {
      Listen();
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
  uint8_t Connect(const char* port, uint32_t baud_rate);
  uint8_t Disconnect() { Serial.end(); return RETURN_OK; }
  void flush() { Serial.flush(); }
#endif

protected:
  /**Process a received command. Derived classes should reimplement this
  function to handle additional commands and they should also make sure to
  call the base class class method.
  \param cmd command code
  \returns RETURN_OK if successfull
  */
  virtual uint8_t ProcessCommand(uint8_t cmd) = 0;
  uint16_t payload_length() { return payload_length_; }

  // WARNING: The following two functions should only be used if you really
  // know what you are doing!  In most cases you can just use Serialize().
  uint8_t* payload() { return payload_; } // pointer to the payload buffer
  void bytes_written(uint16_t bytes) { bytes_written_+=bytes; }

  template<typename T>
    void Serialize(T data,uint16_t size) {
      Serialize((const uint8_t*)data,size); }
  void Serialize(const uint8_t* u, const uint16_t size);
  void SendReply(const uint8_t return_code);

  template<typename T> void ReadArray(T* array, const uint16_t size) {
#if !( defined(AVR) || defined(__SAM3X8E__) ) 
    LogMessage("","ReadArray()");
#endif
    bytes_read_ += size;
    memcpy(array,payload_+bytes_read_-size,size);
  }
  const char* ReadString();
  uint16_t ReadUint16();
  int16_t ReadInt16();
  uint8_t ReadUint8();
  int8_t ReadInt8();
  float ReadFloat();
  uint8_t WaitForReply();

  /**
  Send a command packet to the Arduino. Prior to calling this function, the
  caller should serialize any data that needs to be included in the payload.
  \returns RETURN_OK if successfull.
  \sa Serialize()
  */
  uint8_t SendCommand(const uint8_t cmd);
  void SendNonBlockingCommand(const uint8_t cmd);
  uint8_t ValidateReply(const uint8_t cmd);

#if !( defined(AVR) || defined(__SAM3X8E__) ) 
  inline void LogMessage(const char* msg,
                         const char* function_name,
                         uint8_t level=5) {
    if(debug_) {
      Logging::LogMessage(level,msg,class_name_.c_str(),function_name);
    }
  }
  inline void LogError(const char* msg, const char* function_name) {
    if(debug_) {
            Logging::LogError(msg,class_name_.c_str(),function_name);
    }
  }
  inline void LogSeparator() { if(debug_) { Logging::LogSeparator(); }}
#endif
  char debug_buffer_[MAX_DEBUG_BUFFER_LENGTH];
  uint16_t debug_buffer_length_;
  uint8_t return_code_; // return code
  uint8_t packet_cmd_; // command

private:
  static const uint8_t FRAME_BOUNDARY =           0x7E;
  static const uint8_t CONTROL_ESCAPE =           0x7D;
  static const uint8_t ESCAPE_XOR =               0x20;

  void SendPreamble(const uint8_t cmd);
  void SendPayload();
  void SendByte(uint8_t b);
  uint16_t UpdateCrc(uint16_t crc, uint8_t data);
  void ProcessSerialInput(const uint8_t byte);
  void ProcessPacket();

  uint8_t payload_[MAX_PAYLOAD_LENGTH+2]; // payload (+1 or 2 bytes for payload
                                          // length)
  uint16_t payload_length_; // length of the payload
  uint8_t header_length_; // length of the packet header (2 if payload is
                          // <128 bytes, 3 otherwise)
  uint16_t bytes_received_; // bytes received so far in packet
  uint16_t bytes_read_; // bytes that have been read (by Read methods)
  uint16_t bytes_written_; // bytes that have been written (by Serialize method)
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
