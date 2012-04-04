"""
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
"""

import time
import sys
import math
import re

from path import path
import numpy as np

from .__init__ import package_path
from .__init__ import logger
from dmf_control_board_base import DmfControlBoard as Base
from dmf_control_board_base import uint8_tVector, INPUT, OUTPUT, HIGH, LOW, SINE, SQUARE
from serial_device import SerialDevice, ConnectionError
from avr import AvrDude


class DmfControlBoard(Base, SerialDevice):
    def __init__(self):
        Base.__init__(self)
        SerialDevice.__init__(self)
        self.R_hv = []
        self.C_hv = []
        self.R_fb = []
        self.C_fb = []
                
    def connect(self, port=None):
        if port:
            Base.connect(self, port)
            self.port = port
        else:
            self.get_port()
        logger.info("Poll control board for series resistors and "
                    "capacitance values.")            
        
        self.R_hv = []
        self.C_hv = []
        self.R_fb = []
        self.C_fb = []
        try:
            i=0
            while True:
                self.set_series_resistor_index(0, i)
                self.R_hv.append(
                    self.series_resistance(0))
                self.C_hv.append(
                    self.series_capacitance(0))
                i+=1
        except:
            logger.info("HV series resistors=%s" % self.R_hv)
            logger.info("HV series capacitance=%s" % self.C_hv)
        try:
            i=0
            while True:
                self.set_series_resistor_index(1, i)
                self.R_fb.append(
                    self.series_resistance(1))
                self.C_fb.append(
                    self.series_capacitance(1))
                i+=1
        except:
            logger.info("Feedback series resistors=%s" % self.R_fb)
            logger.info("Feedback series capacitance=%s" % self.C_fb)
        self.set_series_resistor_index(0,0)
        self.set_series_resistor_index(1,0)
        return self.RETURN_OK
    
    @property
    def state_of_all_channels(self):
        return np.array(Base.state_of_all_channels(self))

    @state_of_all_channels.setter
    def state_of_all_channels(self, state):
        state_ = uint8_tVector()
        for i in range(0, len(state)):
            state_.append(int(state[i]))
        Base.set_state_of_all_channels(self, state_)

    @property
    def default_pin_modes(self):
        pin_modes = []
        for i in range(0,53/8+1):
            mode = self.eeprom_read(self.EEPROM_PIN_MODE_ADDRESS+i)
            for j in range(0,8):
                if i*8+j<=53:
                    pin_modes.append(~mode>>j&0x01)
        return pin_modes
        
    @default_pin_modes.setter
    def default_pin_modes(self, pin_modes):
        for i in range(0,53/8+1):
            mode = 0
            for j in range(0,8):
                if i*8+j<=53:
                    mode += pin_modes[i*8+j]<<j
            self.eeprom_write(self.EEPROM_PIN_MODE_ADDRESS+i,~mode&0xFF)
            
    @property
    def default_pin_states(self):
        pin_states = []
        for i in range(0,53/8+1):
            state = self.eeprom_read(self.EEPROM_PIN_STATE_ADDRESS+i)
            for j in range(0,8):
                if i*8+j<=53:
                    pin_states.append(~state>>j&0x01)
        return pin_states
        
    @default_pin_states.setter
    def default_pin_states(self, pin_states):
        for i in range(0,53/8+1):
            state = 0
            for j in range(0,8):
                if i*8+j<=53:
                    state += pin_states[i*8+j]<<j
            self.eeprom_write(self.EEPROM_PIN_STATE_ADDRESS+i,~state&0xFF)

    def analog_reads(self, pin, n_samples):
        return np.array(Base.analog_reads(self, pin, n_samples))

    def sample_voltage(self, adc_channel, n_samples, n_sets,
                       delay_between_sets_ms, state):
        state_ = uint8_tVector()
        for i in range(0, len(state)):
            state_.append(int(state[i]))
        adc_channel_ = uint8_tVector()
        for i in range(0, len(adc_channel)):
            adc_channel_.append(int(adc_channel[i]))
        return np.array(Base.sample_voltage(self,
                        adc_channel_, n_samples, n_sets,
                        delay_between_sets_ms,
                        state_))
    
    def measure_impedance(self, sampling_time_ms, n_samples,
                          delay_between_samples_ms, state):
        state_ = uint8_tVector()
        for i in range(0, len(state)):
            state_.append(int(state[i]))
        impedance = np.array(Base.measure_impedance(self,
                                sampling_time_ms, n_samples,
                                delay_between_samples_ms, state_))
        
        V_hv = impedance[0::4]*5.0/1024/np.sqrt(2)/2
        hv_resistor = impedance[1::4]
        V_fb = impedance[2::4]*5.0/1024/np.sqrt(2)/2
        fb_resistor = impedance[3::4]
        return (V_hv, hv_resistor, V_fb, fb_resistor)
        
    def i2c_write(self, address, data):
        data_ = uint8_tVector()
        for i in range(0, len(data)):
            data_.append(int(data[i]))
        Base.i2c_write(self, address, data_)
        
    def i2c_read(self, address, send_data, n_bytes_to_read):
        send_data_ = uint8_tVector()
        for i in range(0, len(send_data)):
            send_data_.append(int(send_data[i]))
        return np.array(Base.i2c_read(self, address, send_data_, n_bytes_to_read))

    def test_connection(self, port):
        try:
            if self.connect(port) == self.RETURN_OK:
                return True
        except Exception, why:
            if re.search(r'Remote device is not a Arduino DMF ' \
                        'Controller version .*', str(why)):
                logger.warning('On port %s, %s' % (port, why))
        return False
    
    def flash_firmware(self):
        logger.info("[DmfControlBoard].flash_firmware()")
        reconnect = self.connected()  
        if reconnect:
            self.disconnect()
        try:
            hex_path = package_path() / path("dmf_driver.hex")
            logger.info("hex_path=%s" % hex_path)

            logger.info("initializing avrdude")
            avrdude = AvrDude(self.port)

            logger.info("flashing firmware")
            stdout, stderr = avrdude.flash(hex_path.abspath())

            if stdout:
                logger.info(str(stdout))
            if stderr:
                logger.info(str(stderr))
            if reconnect:
                # need to sleep here, otherwise reconnect fails
                time.sleep(.1)
                self.connect(self.port)
        except Exception, why:
            print "Exception flashing firmware: %s" % why
            if reconnect:
                self.connect(self.port)
            raise
