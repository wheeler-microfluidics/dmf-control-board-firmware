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

#include "RemoteObject.h"

#ifdef AVR
#include <util/crc16.h>
#include "WProgram.h"
extern "C" void __cxa_pure_virtual(void); // These declarations are needed for
void __cxa_pure_virtual(void) {}          // virtual functions on the Arduino.
#else
#include <boost/thread.hpp>
#include <boost/timer.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <exception>
using namespace std;

char RemoteObject::log_message_string_[MAX_STRING_SIZE];
#endif

RemoteObject::RemoteObject(uint32_t baud_rate,
                               bool crc_enabled
#ifndef AVR
                               ,const char* class_name //used for logging
#endif
                               ) : baud_rate_(baud_rate),
                                crc_enabled_(crc_enabled)
#ifndef AVR
                                ,class_name_(class_name)
#endif
                                {
  bytes_received_ = 0;
  un_escaping_ = false;
  payload_length_ = 0;
  bytes_read_ = 0;
  bytes_written_ = 0;
  debug_ = false;

#ifdef AVR
  // initialize pin mode and state of digital pins
  // from EEPROM
  for(uint8_t i=0; i<=54/8; i++) {
    uint8_t mode = EEPROM.read(EEPROM_PIN_MODE_ADDRESS+i);
    uint8_t state = EEPROM.read(EEPROM_PIN_STATE_ADDRESS+i);
    for(uint8_t j=0; j<8; j++) {
      if(i*8+j<54) {
        pinMode(i*8+j,(~mode>>j)&0x01);
        digitalWrite(i*8+j,(~state>>j)&0x01);
      }
    }
  }
#endif
}

RemoteObject::~RemoteObject() {
}

void RemoteObject::SendByte(const uint8_t b) {
#ifndef AVR
  const char* function_name = "SendByte()";
#endif
  if(b==FRAME_BOUNDARY || b==CONTROL_ESCAPE) {
#ifndef AVR
    sprintf(log_message_string_,"write escape (0x%0X)",b);
    LogMessage(log_message_string_,function_name);
#endif
    Serial.write(CONTROL_ESCAPE);
    Serial.write(b^ESCAPE_XOR);
  } else {
#ifndef AVR
    sprintf(log_message_string_,"write (0x%0X)",b);
    LogMessage(log_message_string_,function_name);
#endif
    Serial.write(b);
  }
}

uint16_t RemoteObject::UpdateCrc(uint16_t crc, uint8_t data) {
#ifdef AVR
  crc = _crc16_update(crc,data);
#else
  crc ^= data;
  for(uint8_t i=0; i<8; i++) {
    if(crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
#endif
  return crc;
}

void RemoteObject::SendPreamble() {
  payload_length_ = bytes_written_;
#ifndef AVR
  const char* function_name = "SendPreamble()";
  sprintf(log_message_string_,
          "command=0x%0X (%d), payload_length=%d",
          packet_cmd_,packet_cmd_,payload_length_);
  LogMessage(log_message_string_,function_name);
#endif
  Serial.write(FRAME_BOUNDARY);
  if(crc_enabled_) {
    tx_crc_ = 0xFFFF; // reset crc
    tx_crc_ = UpdateCrc(tx_crc_, packet_cmd_);
  }
  SendByte(packet_cmd_);
  if(payload_length_<128) {
    if(crc_enabled_) {
      tx_crc_ = UpdateCrc(tx_crc_, (uint8_t)payload_length_);
    }
    SendByte((uint8_t)payload_length_);
  } else {
    if(crc_enabled_) {
      tx_crc_ = UpdateCrc(tx_crc_, (uint8_t)((0x8000|payload_length_)>>8));
      tx_crc_ = UpdateCrc(tx_crc_, (uint8_t)payload_length_);
    }
    SendByte((uint8_t)((0x8000|payload_length_)>>8));
    SendByte((uint8_t)payload_length_);
  }
}

uint8_t RemoteObject::SendCommand(const uint8_t cmd) {
#ifndef AVR
  const char* function_name = "SendCommand()";
  LogSeparator();
  LogMessage("",function_name);
  time_cmd_sent_ = boost::posix_time::microsec_clock::universal_time();
#endif
  packet_cmd_ = cmd;
  SendPreamble();
  SendPayload();
  return_code_ = WaitForReply();
#ifndef AVR
  sprintf(log_message_string_,"return code=%d, cmd returned in %d us",
          return_code_, (boost::posix_time::microsec_clock::universal_time()
                         -time_cmd_sent_).total_microseconds());
  LogMessage(log_message_string_, function_name);
  if(return_code_!=RETURN_OK) {
    //LogMessage("Throw exception",function_name);
    //throw runtime_error("Communication error.");
  }
#endif
  return return_code_;
}

void RemoteObject::Serialize(const uint8_t* u,const uint16_t size) {
#ifndef AVR
  const char* function_name = "Serialize()";
  sprintf(log_message_string_,"%d bytes.",size);
  LogMessage(log_message_string_, function_name);
#endif
  //TODO check that MAX_PAYLOAD_LENGTH isn't exceeded
  for(uint16_t i=0;i<size;i++) {
#ifndef AVR
    sprintf(log_message_string_,"(0x%0X) byte %d",u[i],i);
    LogMessage(log_message_string_, function_name);
#endif
    payload_[bytes_written_+i]=u[i];
  }
  bytes_written_+=size;
}

void RemoteObject::SendPayload() {
#ifndef AVR
  const char* function_name = "SendPayload()";
  sprintf(log_message_string_,"%d bytes",payload_length_);
  LogMessage(log_message_string_, function_name);
#endif
  for(uint16_t i=0; i<payload_length_; i++) {
    if(crc_enabled_) {
      tx_crc_ = UpdateCrc(tx_crc_, payload_[i]);
    }
    SendByte(payload_[i]);
  }
  if(crc_enabled_) {
    SendByte((uint8_t)tx_crc_);
    SendByte((uint8_t)(tx_crc_>>8));
  }
  payload_length_ = 0;
  bytes_written_ = 0;
}

void RemoteObject::SendReply(uint8_t return_code) {
  Serialize(&return_code,1);
  SendPreamble();
  SendPayload();
}

const char* RemoteObject::ReadString() {
  const char* function_name = "ReadString()";
  // TODO check that we're not reading past the end of the buffer
  uint8_t length = strlen((const char*)payload_)+1;
  bytes_read_ += length;
#ifndef AVR
  sprintf(log_message_string_,
          "=\"%s\", bytes_read_=%d",
          (const char*)(payload_+bytes_read_-length),
          bytes_read_);
  LogMessage(log_message_string_, function_name);
#endif
  return (const char*)(payload_+bytes_read_-length);
}

uint8_t RemoteObject::ReadUint8() {
  bytes_read_ += sizeof(uint8_t);
#ifndef AVR
  const char* function_name = "ReadUint8()";
  sprintf(log_message_string_,
          "=%d, bytes_read_=%d",
          *(uint8_t*)(payload_+bytes_read_-sizeof(uint8_t)),
          bytes_read_);
  LogMessage(log_message_string_, function_name);
#endif
  return *(uint8_t*)(payload_+bytes_read_-sizeof(uint8_t));
}

uint16_t RemoteObject::ReadUint16() {
  bytes_read_ += sizeof(uint16_t);
#ifndef AVR
  const char* function_name = "ReadUint16()";
  sprintf(log_message_string_,
          "=%d, bytes_read_=%d",
          *(uint16_t*)(payload_+bytes_read_-sizeof(uint16_t)),
          bytes_read_);
  LogMessage(log_message_string_, function_name);
#endif
  return *(uint16_t*)(payload_+bytes_read_-sizeof(uint16_t));
}

float RemoteObject::ReadFloat() {
  bytes_read_ += sizeof(float);
#ifndef AVR
  const char* function_name = "ReadFloat()";
  sprintf(log_message_string_,
          "=%.1f, bytes_read_=%d",
          *(float*)(payload_+bytes_read_-sizeof(float)),
          bytes_read_);
  LogMessage(log_message_string_, function_name);
#endif
  return *(float*)(payload_+bytes_read_-sizeof(float));
}

uint8_t RemoteObject::WaitForReply() {
#ifndef AVR
  LogMessage("", "WaitForReply()");
#endif
  char b;
  waiting_for_reply_to_ = packet_cmd_;
  while(waiting_for_reply_to_) {
    if(Serial.available()) {
      b = Serial.read();
      ProcessSerialInput(b);
    }
#ifndef AVR
      else if((boost::posix_time::microsec_clock::universal_time()
       -time_cmd_sent_).total_microseconds()>TIMEOUT_MICROSECONDS) {
      return_code_ = RETURN_TIMEOUT;
      waiting_for_reply_to_ = 0;
    }
#endif
  }
  return return_code_;
}

void RemoteObject::ProcessPacket() {
  if(packet_cmd_&0x80) { // Commands have MSB==1
    packet_cmd_ = packet_cmd_^0x80; // Flip the MSB for reply
    ProcessCommand(packet_cmd_^0x80);
#ifndef AVR
    LogSeparator();
#endif
  } else {
    return_code_ = payload_[payload_length_-1];
    payload_length_--;// -1 because we've already read the return code
#ifndef AVR
  const char* function_name = "ProcessPacket()";
  sprintf(log_message_string_,
          "(0x%0X). This packet is a reply to command (%d)",
          packet_cmd_^0x80,packet_cmd_^0x80);
  LogMessage(log_message_string_, function_name);
  sprintf(log_message_string_,"Return code=%d",return_code());
  LogMessage(log_message_string_, function_name);
  sprintf(log_message_string_,"Payload length=%d",payload_length());
  LogMessage(log_message_string_, function_name);
  LogSeparator();
#endif
  }
}

uint8_t RemoteObject::ProcessCommand(uint8_t cmd) {
#ifndef AVR
  const char* function_name = "ProcessCommand()";
  sprintf(log_message_string_,"command=0x%0X (%d)",
          cmd,cmd);
  LogMessage(log_message_string_, function_name);
#endif
  uint8_t return_code = RETURN_UNKNOWN_COMMAND;
  switch(cmd) {
#ifdef AVR // Commands that only the Arduino handles
    case CMD_SET_PIN_MODE:
      if(payload_length()==2) {
        uint8_t pin = ReadUint8();
        uint8_t mode = ReadUint8();
        pinMode(pin, mode);
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_DIGITAL_WRITE:
      if(payload_length()==2) {
        uint8_t pin = ReadUint8();
        uint8_t value = ReadUint8();
        digitalWrite(pin, value);
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_DIGITAL_READ:
      if(payload_length()==1) {
        uint8_t pin = ReadUint8();
        uint8_t value = digitalRead(pin);
        Serialize(&value,sizeof(value));
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_ANALOG_WRITE:
      if(payload_length()==2) {
        uint8_t pin = ReadUint8();
        uint16_t value = ReadUint16();
        analogWrite(pin, value);
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_ANALOG_READ:
      if(payload_length()==1) {
        uint8_t pin = ReadUint8();
        uint16_t value = analogRead(pin);
        Serialize(&value,sizeof(value));
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_EEPROM_WRITE:
      if(payload_length()==3) {
        uint16_t address = ReadUint16();
        uint8_t value = ReadUint8();
        EEPROM.write(address, value);
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_EEPROM_READ:
      if(payload_length()==2) {
        uint16_t address = ReadUint16();
        uint8_t value = EEPROM.read(address);
        Serialize(&value,sizeof(value));
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_ONEWIRE_GET_ADDRESS:
      if(payload_length()==2) {
        uint8_t pin = ReadUint8();
        uint8_t index = ReadUint8();
        uint8_t addr[8];
        uint8_t ret;
        OneWire ow(pin);
        ow.reset_search();
        for(uint8_t i=0; i<=index; i++) {
          ret = ow.search(addr);
        }
        if(ret) {
          Serialize(addr, 8*sizeof(uint8_t));
          return_code = RETURN_OK;
        } else { // No more addresses
          return_code = RETURN_BAD_INDEX;
        }
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_ONEWIRE_READ:
      if(payload_length()==11) {
        uint8_t pin = ReadUint8();
        uint8_t addr[8];
        for(uint8_t i=0; i<8; i++) {
          addr[i] = ReadUint8();
        }
        uint8_t command = ReadUint8();
        uint8_t n_bytes = ReadUint8();
        OneWire ow(pin);
        ow.reset();
        ow.select(addr);
        ow.write(command);
        for(uint8_t i=0; i<n_bytes; i++) {
          uint8_t data = ow.read();
          Serialize(&data, sizeof(uint8_t));
        }
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
    case CMD_ONEWIRE_WRITE:
      if(payload_length()==11) {
        uint8_t pin = ReadUint8();
        uint8_t addr[8];
        for(uint8_t i=0; i<8; i++) {
          addr[i] = ReadUint8();
        }
        uint8_t value = ReadUint8();
        uint8_t power = ReadUint8();
        OneWire ow(pin);
        ow.reset();
        ow.select(addr);
        ow.write(value,power);
        return_code = RETURN_OK;
      } else {
        return_code = RETURN_BAD_PACKET_SIZE;
      }
      break;
#endif
  }
  return return_code;
}

void RemoteObject::ProcessSerialInput(uint8_t b) {
#ifndef AVR
  const char* function_name = "ProcessSerialInput()";
#endif
  // deal with escapes
  if (b==CONTROL_ESCAPE) {
#ifndef AVR
    sprintf(log_message_string_,"(0x%0X) Escape",b);
    LogMessage("", function_name);
#endif
    un_escaping_ = true;
    return;
  } else if(un_escaping_) {
    b^=ESCAPE_XOR;
#ifndef AVR
    sprintf(log_message_string_,
            "(0x%0X) Un-escaping",b);
    LogMessage(log_message_string_, function_name);
#endif
  }
  if (b==FRAME_BOUNDARY && !un_escaping_) {
#ifndef AVR
    LogSeparator();
    sprintf(log_message_string_,
            "(0x%0X) Frame Boundary",b);
    LogMessage(log_message_string_, function_name);
#endif
    if(bytes_received_>0) {
#ifndef AVR
      sprintf(log_message_string_,"(0x%0X) Invalid packet",b);
      LogMessage(log_message_string_, function_name);
#endif
    }
    bytes_received_ = 0;
  } else {
    if(bytes_received_==0) { // command byte
#ifndef AVR
      sprintf(log_message_string_,
              "(0x%0X) Command byte (%d)",b,b);
      LogMessage(log_message_string_, function_name);
#endif
      packet_cmd_=b;
      if(crc_enabled_) {
        rx_crc_=0xFFFF; // reset the crc
      }
    } else if(bytes_received_==1) { // payload length
      if(b & 0x80) {
        header_length_=3;
        payload_length_=(b&0x7F)<<8;
      } else {
        header_length_=2;
        payload_length_=b;
      }
    // payload length (byte 2)
    } else if(bytes_received_==2 && header_length_==3) {
      payload_length_+=b;
    } else if(bytes_received_-header_length_<payload_length_) { // payload
      // TODO: check that MAX_PAYLOAD_LENGTH isn't exceeded
      payload_[bytes_received_-header_length_]=b;
    } else if(bytes_received_-header_length_<payload_length_+2) { // crc
    } else {
      // TODO: error
    }
#ifndef AVR
    if(bytes_received_==header_length_) {
      sprintf(log_message_string_, "Payload length=%d", payload_length_);
      LogMessage(log_message_string_, function_name);
    }
#endif
    if(crc_enabled_) {
      rx_crc_ = UpdateCrc(rx_crc_, b);
    }
    bytes_received_++;
#ifndef AVR
    if(b>=0x20&&b<=0x7E) {
      sprintf(log_message_string_,
              "(0x%0X) %d bytes received (\'%c\')",
              b, bytes_received_ ,b);
    } else {
      sprintf(log_message_string_,
              "(0x%0X) %d bytes received",
              b, bytes_received_);
    }
    LogMessage(log_message_string_, function_name);
#endif
    if(bytes_received_==payload_length_+header_length_+2*crc_enabled_) {
      bytes_received_ = 0;
      bytes_read_ = 0;
      bytes_written_ = 0;
      if(crc_enabled_) {
        if(rx_crc_==0) {
#ifndef AVR
          LogMessage("End of Packet. CRC OK.", function_name);
#endif
          ProcessPacket();
        } else {
#ifndef AVR
          LogMessage("End of Packet. CRC Error.", function_name);
#endif
        }
      } else {
#ifndef AVR
        LogMessage("End of Packet", function_name);
#endif
        ProcessPacket();
      }
#ifndef AVR
      LogSeparator();
#endif
      // if we're not expecting something else, stop waiting
      if(packet_cmd_==(waiting_for_reply_to_^0x80)) {
        waiting_for_reply_to_ = 0;
      }
#ifndef AVR
      else {
        LogMessage("Not the expected reply, keep waiting.", function_name);
      }
#endif
    }
  }
  if(un_escaping_) {
    un_escaping_=false;
  }
}

#ifdef AVR
////////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the Arduino
//
////////////////////////////////////////////////////////////////////////////////

void RemoteObject::begin() {
  Serial.begin(57600);
}

void RemoteObject::Listen() {
  while(Serial.available()>0) {
    ProcessSerialInput(Serial.read());
  }
}
#else
////////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the PC
//
////////////////////////////////////////////////////////////////////////////////

uint8_t RemoteObject::Connect(const char* port) {
  const char* function_name = "Connect()";
  int return_code = Serial.begin(port, baud_rate_);

  // wait up to 10 s for the Arduino to send something on the
  // serial port so that we know it's ready
  if(return_code==RETURN_OK) {
    boost::posix_time::ptime t = 
      boost::posix_time::microsec_clock::universal_time();
    while(Serial.available()==false && \
      (boost::posix_time::microsec_clock::universal_time()-t)
      .total_seconds()<10) {
    }
  }
  sprintf(log_message_string_,"Serial.begin(%s,%d)=%d",
          port,baud_rate_,return_code);
  LogMessage(log_message_string_, function_name);
  return return_code;
}

void RemoteObject::set_debug(const bool debug) {
  debug_ = debug;
}

string RemoteObject::protocol_name() {
  const char* function_name = "protocol_name()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(SendCommand(CMD_GET_PROTOCOL_NAME)==RETURN_OK) {
    string protocol_name = ReadString();
    sprintf(log_message_string_,
            "protocol_name=%s",
            protocol_name.c_str());
    LogMessage(log_message_string_, function_name);
    return protocol_name;
  }
  return "";
}

string RemoteObject::protocol_version() {
  const char* function_name = "protocol_version()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(SendCommand(CMD_GET_PROTOCOL_VERSION)==RETURN_OK) {
    string protocol_version = ReadString();
    sprintf(log_message_string_,
            "protocol_version=%s",
            protocol_version.c_str());
    LogMessage(log_message_string_, function_name);
    return protocol_version;
  }
  return "";
}

string RemoteObject::name() {
  const char* function_name = "name()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(SendCommand(CMD_GET_DEVICE_NAME)==RETURN_OK) {
    string name = ReadString();
    sprintf(log_message_string_,
            "name=%s",
            name.c_str());
    LogMessage(log_message_string_, function_name);
    return name;
  }
  return "";
}

string RemoteObject::manufacturer() {
  const char* function_name = "manufacturer()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(SendCommand(CMD_GET_MANUFACTURER)==RETURN_OK) {
    string manufacturer = ReadString();
    sprintf(log_message_string_,
            "manufacturer=%s",
            manufacturer.c_str());
    LogMessage(log_message_string_, function_name);
    return manufacturer;
  }
  return "";
}

string RemoteObject::software_version() {
  const char* function_name = "software_version()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(SendCommand(CMD_GET_SOFTWARE_VERSION)==RETURN_OK) {
    string software_version = ReadString();
    sprintf(log_message_string_,
            "software_version=%s",
            software_version.c_str());
    LogMessage(log_message_string_, function_name);
    return software_version;
  }
  return "";
}

string RemoteObject::hardware_version() {
  const char* function_name = "hardware_version()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(SendCommand(CMD_GET_HARDWARE_VERSION)==RETURN_OK) {
    string hardware_version = ReadString();
    sprintf(log_message_string_,
            "hardware_version=%s",
            hardware_version.c_str());
    LogMessage(log_message_string_, function_name);
    return hardware_version;
  }
  return "";
}

string RemoteObject::url() {
  const char* function_name = "url()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(SendCommand(CMD_GET_URL)==RETURN_OK) {
    string url = ReadString();
    sprintf(log_message_string_,
            "url=%s",
            url.c_str());
    LogMessage(log_message_string_, function_name);
    return url;
  }
  return "";
}

void RemoteObject::set_pin_mode(uint8_t pin, uint8_t mode) {
  const char* function_name = "set_pin_mode()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&pin,sizeof(pin));
  Serialize(&pin,sizeof(mode));
  if(SendCommand(CMD_SET_PIN_MODE)==RETURN_OK) {
    sprintf(log_message_string_,"pin %d mode=%d",
            pin, mode);
    LogMessage(log_message_string_, function_name);
  }
}

uint8_t RemoteObject::digital_read(uint8_t pin) {
  const char* function_name = "digital_read()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&pin,sizeof(pin));
  if(SendCommand(CMD_DIGITAL_READ)==RETURN_OK) {
    uint8_t value = ReadUint8();
    sprintf(log_message_string_,"pin %d value=%d",
            pin, value);
    LogMessage(log_message_string_, function_name);
    return value;
  }
  return 0;
}

void RemoteObject::digital_write(uint8_t pin, uint8_t value) {
  const char* function_name = "digital_write()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&pin,sizeof(pin));
  Serialize(&value,sizeof(value));
  if(SendCommand(CMD_DIGITAL_WRITE)==RETURN_OK) {
    sprintf(log_message_string_,"pin %d value=%d",
            pin, value);
    LogMessage(log_message_string_, function_name);
  }
}

uint16_t RemoteObject::analog_read(uint8_t pin) {
  const char* function_name = "analog_read()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&pin,sizeof(pin));
  if(SendCommand(CMD_ANALOG_READ)==RETURN_OK) {
    uint16_t value = ReadUint16();
    sprintf(log_message_string_,"pin %d value=%d",
            pin, value);
    LogMessage(log_message_string_, function_name);
    return value;
  }
  return 0;
}

void RemoteObject::analog_write(uint8_t pin, uint16_t value) {
  const char* function_name = "analog_write()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&pin,sizeof(pin));
  Serialize(&value,sizeof(value));
  if(SendCommand(CMD_DIGITAL_WRITE)==RETURN_OK) {
    sprintf(log_message_string_,"pin %d value=%d",
            pin, value);
    LogMessage(log_message_string_, function_name);
  }
}

uint8_t RemoteObject::eeprom_read(uint16_t address) {
  const char* function_name = "eeprom_read()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&address,sizeof(address));
  if(SendCommand(CMD_EEPROM_READ)==RETURN_OK) {
    uint8_t value = ReadUint8();
    sprintf(log_message_string_,"address %d value=%d",
            address, value);
    LogMessage(log_message_string_, function_name);
    return value;
  }
  return 0;
}

void RemoteObject::eeprom_write(uint16_t address, uint8_t value) {
  const char* function_name = "eeprom_write()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&address,sizeof(address));
  Serialize(&value,sizeof(value));
  if(SendCommand(CMD_EEPROM_WRITE)==RETURN_OK) {
    sprintf(log_message_string_,"address %d value=%d",
            address, value);
    LogMessage(log_message_string_, function_name);
  }
}

std::vector<uint8_t> RemoteObject::onewire_address(uint8_t pin,
                                                   uint8_t index) {
  const char* function_name = "onewire_address()";
  LogSeparator();
  LogMessage("send command", function_name);
  Serialize(&pin, sizeof(pin));
  Serialize(&index, sizeof(index));
  if(SendCommand(CMD_ONEWIRE_GET_ADDRESS)==RETURN_OK) {
    sprintf(log_message_string_,"pin %d, index=%d",
            pin, index);
    LogMessage(log_message_string_, function_name);
    std::vector<uint8_t> address;
    for(int i=0; i<payload_length(); i++) {
      address.push_back(ReadUint8());
    }
    return address;
  }
  return std::vector<uint8_t>();
}

std::vector<uint8_t> RemoteObject::onewire_read(uint8_t pin,
                                                std::vector<uint8_t> address,
                                                uint8_t command,
                                                uint8_t n_bytes) {
  const char* function_name = "onewire_read()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(address.size()==8) {
    Serialize(&pin,sizeof(pin));
    Serialize(&address[0],8*sizeof(uint8_t));
    Serialize(&command,sizeof(command));
    Serialize(&n_bytes,sizeof(n_bytes));
    if(SendCommand(CMD_ONEWIRE_READ)==RETURN_OK) {
      std::vector<uint8_t> data;
      for(uint8_t i=0; i<n_bytes; i++) {
        data.push_back(ReadUint8());
      }
      sprintf(log_message_string_,"pin %d, command=%d, n_bytes=%d",
              pin, command, n_bytes);
      LogMessage(log_message_string_, function_name);
      return data;
    }
  }
  return std::vector<uint8_t>();
}

void RemoteObject::onewire_write(uint8_t pin, std::vector<uint8_t> address,
                                 uint8_t value, uint8_t power) {
  const char* function_name = "onewire_write()";
  LogSeparator();
  LogMessage("send command", function_name);
  if(address.size()==8) {
    Serialize(&pin,sizeof(pin));
    Serialize(&address[0],8*sizeof(uint8_t));
    Serialize(&value,sizeof(value));
    Serialize(&power,sizeof(power));
    if(SendCommand(CMD_ONEWIRE_WRITE)==RETURN_OK) {
      sprintf(log_message_string_,
              "pin %d, value=%d, power=%d",
              pin, value, power);
    }
    LogMessage(log_message_string_, function_name);
  }
}

#endif
