/*
RemoteObject

This class implements a simple communication protocol between a PC and
Arduino over an RS-232 link using HDLC-like framing.  We use a wrapper
for the boost asio library (SimpleSerial) that provides an interface that
closely matches the Serial library on the Arduino.  This allows us to share
the bulk of the code between the PC (Windows, Linux or Mac) and the Arduino.

This implementation was partly inspired by Alvaro Lopes' serpro project:
  https://github.com/alvieboy/arduino-serpro

Each packet has the following structure:

+------------+---------+----------------+---------+------------+----------+
| Start Flag | Command | Payload Length | Payload |    CRC     | End Flag |
|   1 byte   | 1 byte  |    1-2 bytes   | N bytes |  2 bytes   |  1 byte  |
|    0x7E    |         |                |         | (optional) |   0x7E   |
+------------+---------+----------------+---------+------------+----------+

The payload length can be one or two bytes.  If the payload is less than 128
bytes, it's length is expressed as a single byte.  If the most-significant
bit is set, the length is expressed as two bytes and can be recovered by
clearing the most significant byte (i.e. PAYLOAD_LENGTH & 0x7FFF).

 Examples:

   payload length of 3, one byte: 0x04
   payload length of 512, two bytes: 0x82 0x01

Total packet length (not including flags) = Header Length (2-3 bytes)
                                            + Payload Length
                                            (+ 2 if CRC is enabled)

To use this class, you must derive a class based on it and reimplement the
virtual member function "ProcessPacket(...)".


________________________________________________________________________

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

#ifndef _REMOTE_OBJECT_H
#define	_REMOTE_OBJECT_H

#include <stdint.h>

#ifdef AVR
  #include <EEPROM.h>
  #include <OneWire.h>
#else
  #include "logging.h"
  #include "SimpleSerial.h"
  #include <string>
#endif

class RemoteObject {
public:
#ifndef AVR
  static const uint32_t TIMEOUT_MICROSECONDS =   2000000; // TODO: this should be configurable
#endif
  // EEPROM addresses
  static const uint16_t EEPROM_PIN_MODE_ADDRESS =      0;
  static const uint16_t EEPROM_PIN_STATE_ADDRESS =     7;

  // protocol constants
  static const uint16_t MAX_PAYLOAD_LENGTH =        2001;
  static const uint8_t MAX_STRING_SIZE =              80;

  // reserved commands
  static const uint8_t CMD_GET_PROTOCOL_NAME =      0x80;
  static const uint8_t CMD_GET_PROTOCOL_VERSION =   0x81;
  static const uint8_t CMD_GET_DEVICE_NAME =        0x82;
  static const uint8_t CMD_GET_MANUFACTURER =       0x83;
  static const uint8_t CMD_GET_HARDWARE_VERSION =   0x84;
  static const uint8_t CMD_GET_SOFTWARE_VERSION =   0x85;
  static const uint8_t CMD_GET_URL =                0x86;
  static const uint8_t CMD_SET_PIN_MODE =           0x87;
  static const uint8_t CMD_DIGITAL_READ =           0x88;
  static const uint8_t CMD_DIGITAL_WRITE =          0x89;
  static const uint8_t CMD_ANALOG_READ =            0x8A;
  static const uint8_t CMD_ANALOG_WRITE =           0x8B;
  static const uint8_t CMD_EEPROM_READ =            0x8C;
  static const uint8_t CMD_EEPROM_WRITE =           0x8D;
  static const uint8_t CMD_ONEWIRE_GET_ADDRESS =    0x8E;
  static const uint8_t CMD_ONEWIRE_WRITE =          0x8F;
  static const uint8_t CMD_ONEWIRE_READ =           0x90;

  // reserved return codes
  static const uint8_t RETURN_OK =                  0x00;
  static const uint8_t RETURN_GENERAL_ERROR =       0x01;
  static const uint8_t RETURN_UNKNOWN_COMMAND =     0x02;
  static const uint8_t RETURN_TIMEOUT =             0x03;
  static const uint8_t RETURN_NOT_CONNECTED =       0x04;
  static const uint8_t RETURN_BAD_INDEX =           0x05;
  static const uint8_t RETURN_BAD_PACKET_SIZE =     0x06;
  static const uint8_t RETURN_BAD_CRC =             0x07;

  RemoteObject(uint32_t baud_rate,
                 bool crc_enabled_
#ifndef AVR
                 ,const char* class_name
#endif
                 );
  ~RemoteObject();

  uint8_t return_code() {return return_code_; }
  bool crc_enabled() { return crc_enabled_; }

#ifdef AVR
  void Listen();
  virtual void begin();

  // these methods force the derived class to define functions that
  // return the following attributes
  virtual const char* protocol_name() = 0;
  virtual const char* protocol_version() = 0;
  virtual const char* name() = 0;
  virtual const char* manufacturer() = 0;
  virtual const char* software_version() = 0;
  virtual const char* hardware_version() = 0;
  virtual const char* url() = 0;
#else
  virtual std::string host_software_version() = 0;

  // Remote accessors
  std::string protocol_name();
  std::string protocol_version();
  std::string name();
  std::string manufacturer();
  std::string software_version();
  std::string hardware_version();
  std::string url();
  void set_pin_mode(uint8_t pin, uint8_t mode);
  uint8_t digital_read(uint8_t pin);
  void digital_write(uint8_t pin, uint8_t value);
  uint16_t analog_read(uint8_t pin);
  void analog_write(uint8_t pin, uint16_t value);
  uint8_t eeprom_read(uint16_t address);
  void eeprom_write(uint16_t address, uint8_t value);
  std::vector<uint8_t> onewire_address(uint8_t pin, uint8_t index);
  std::vector<uint8_t> onewire_read(uint8_t pin, std::vector<uint8_t> address,
                                    uint8_t command, uint8_t n_bytes);
  void onewire_write(uint8_t pin, std::vector<uint8_t> address,
                     uint8_t value, uint8_t power);

  void set_debug(const bool debug);
  bool connected() { return Serial.isOpen(); }  
  uint8_t Connect(const char* port);
  void flush() { Serial.flush(); }
#endif

protected:
  // this virtual method must be overriden in the derived class
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
#ifndef AVR
    LogMessage("","ReadArray()");
#endif
    bytes_read_ += size;
    memcpy(array,payload_+bytes_read_-size,size);
  }
  const char* ReadString();
  uint16_t ReadUint16();
  uint8_t ReadUint8();
  float ReadFloat();
  uint8_t WaitForReply();
  uint8_t SendCommand(const uint8_t cmd);

#ifndef AVR
  inline void LogMessage(const char* msg,
                         const char* function_name,
                         uint8_t level=5) {
          if(debug_) {
            Logging::LogMessage(level,msg,class_name_.c_str(),function_name); }}
  inline void LogError(const char* msg, const char* function_name) {
          if(debug_) {
          Logging::LogError(msg,class_name_.c_str(),function_name); }}
  inline void LogSeparator() { if(debug_) { Logging::LogSeparator(); }}
  static char log_message_string_[];
#endif
  uint8_t return_code_; // return code

private:
  static const uint8_t FRAME_BOUNDARY =           0x7E;
  static const uint8_t CONTROL_ESCAPE =           0x7D;
  static const uint8_t ESCAPE_XOR =               0x20;

  void SendPreamble();
  void SendPayload();
  void SendByte(uint8_t b);
  uint16_t UpdateCrc(uint16_t crc, uint8_t data);
  void ProcessPacket();
  void ProcessSerialInput(const uint8_t byte);

  uint8_t packet_cmd_; // command
  uint8_t payload_[MAX_PAYLOAD_LENGTH]; // payload
  uint16_t payload_length_; // length of the payload
  uint8_t header_length_; // length of the packet header (2 if payload is
                          // <128 bytes, 3 otherwise)
  uint32_t baud_rate_;
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
#ifndef AVR
  SimpleSerial Serial;
  std::string class_name_;
  boost::posix_time::ptime time_cmd_sent_;
#endif
};

#endif	// _REMOTE_OBJECT_H
