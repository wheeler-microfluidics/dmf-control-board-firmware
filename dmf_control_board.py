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
import copy

from path import path
import numpy as np

from .__init__ import package_path
from .__init__ import logger
from dmf_control_board_base import DmfControlBoard as Base
from dmf_control_board_base import uint8_tVector, INPUT, OUTPUT, HIGH, LOW, SINE, SQUARE
from serial_device import SerialDevice, ConnectionError
from avr import AvrDude


class FeedbackCalibration():
    def __init__(self, R_hv=None, C_hv=None, R_fb=None, C_fb=None):
        if R_hv:
            self.R_hv = np.array(R_hv)
        else:
            self.R_hv = np.array([8.7e4, 6.4e5])
        if C_hv:
            self.C_hv = np.array(C_hv)
        else:
            self.C_hv = np.array([1.4e-10, 1.69e-10])
        if R_fb:
            self.R_fb = np.array(R_fb)
        else:
            self.R_fb = np.array([1.14e3, 1e4, 9.3e4, 6.5e5])
        if C_fb:
            self.C_fb = np.array(C_fb)
        else:
            self.C_fb = np.array([3e-14, 3.2e-10, 3.3e-10, 3.4e-10])      

    def __getstate__(self):
        """Convert numpy arrays to lists for serialization"""
        out = copy.deepcopy(self.__dict__)
        for k, v in out.items():
            if isinstance(v, np.ndarray):
                out[k] = v.tolist()
        return out

    def __setstate__(self, state):
        """Convert lists to numpy arrays after loading serialized object"""
        self.__dict__ = state
        for k, v in self.__dict__.items():
            if k=='R_hv' or k=='C_hv' or k=='R_fb' or k=='C_fb':
                self.__dict__[k] = np.array(v)


class DmfControlBoard(Base, SerialDevice):
    def __init__(self):
        Base.__init__(self)
        SerialDevice.__init__(self)
        self.calibration = None
                
    def connect(self, port=None):
        if port:
            Base.connect(self, port)
            self.port = port
        else:
            self.get_port()
        logger.info("Poll control board for series resistors and "
                    "capacitance values.")            

        R_hv = []
        C_hv = []
        R_fb = []
        C_fb = []        
        try:
            i=0
            while True:
                self.set_series_resistor_index(0, i)
                R_hv.append(self.series_resistance(0))
                C_hv.append(self.series_capacitance(0))
                i+=1
        except:
            logger.info("HV series resistors=%s" % R_hv)
            logger.info("HV series capacitance=%s" % C_hv)
        try:
            i=0
            while True:
                self.set_series_resistor_index(1, i)
                R_fb.append(self.series_resistance(1))
                C_fb.append(self.series_capacitance(1))
                i+=1
        except:
            logger.info("Feedback series resistors=%s" % R_fb)
            logger.info("Feedback series capacitance=%s" % C_fb)
        self.calibration = FeedbackCalibration(R_hv, C_hv, R_fb, C_fb)
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
