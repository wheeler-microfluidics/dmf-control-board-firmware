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
import os
from collections import OrderedDict
import decorator
import time
import copy
import logging
from struct import pack, unpack
import math

import numpy as np
import matplotlib.mlab as mlab
from path_helpers import path
from microdrop_utility import Version, FutureVersionError

from dmf_control_board_base import DMFControlBoard as Base
from dmf_control_board_base import uint8_tVector
# Import firmware constants
from dmf_control_board_base import INPUT, OUTPUT, HIGH, LOW
from serial_device import SerialDevice
from avr_helpers import AvrDude


logger = logging.getLogger()


def package_path():
    return path(os.path.abspath(os.path.dirname(__file__)))


def get_sketch_directory():
    '''
    Return directory containing the `dmf_control_board` Arduino sketch.
    '''
    return os.path.join(package_path(), 'src', 'dmf_control_board')


def get_includes():
    '''
    Return directories containing the `dmf_control_board` Arduino header files.

    Modules that need to compile against `dmf_control_boad` should use this function
    to locate the appropriate include directories.

    Notes
    =====

    For example:

        import dmf_control_board
        ...
        print ' '.join(['-I%s' % i for i in dmf_control_board.get_includes()])
        ...

    '''
    return [get_sketch_directory()]


def get_firmwares():
    '''
    Return `dmf_control_board` compiled Arduino hex file paths.

    This function may be used to locate firmware binaries that are available
    for flashing to [Arduino Mega2560][1] boards.

    [1]: http://arduino.cc/en/Main/arduinoBoardMega2560
    '''
    return OrderedDict([(board_dir.name, [f.abspath() for f in
                                          board_dir.walkfiles('*.hex')])
                        for board_dir in
                        package_path().joinpath('firmware').dirs()])


def get_sources():
    '''
    Return `dmf_control_board` Arduino source file paths.

    Modules that need to compile against `dmf_control_board` should use this
    function to locate the appropriate source files to compile.

    Notes
    =====

    For example:

        import dmf_control_board
        ...
        print ' '.join(dmf_control_board.get_sources())
        ...

    '''
    return get_sketch_directory().files('*.c*')


def safe_getattr(obj, attr, except_types):
    '''
    Execute `getattr` to retrieve the specified attribute from the provided
    object, returning a default value of `None` in the case where the attribute
    does not exist.

    In the case where an exception occurs during the `getattr` call, if the
    exception type is in `except_types`, ignore the exception and return
    `None`.
    '''
    try:
        return getattr(obj, attr, None)
    except except_types:
        return None


class PersistentSettingDoesNotExist(Exception):
    pass


def feedback_signal(p, frequency, Z, hw_version):
    """p[0]=C, p[1]=R"""
    if hw_version.major == 1:
        return np.abs(1 / (Z / p[1] + 1 + Z * 2 * np.pi * p[0] * complex(0, 1)
                           * frequency))
    else:
        return np.abs(1 / (Z / p[1] + Z * 2 * np.pi * p[0] * complex(0, 1) *
                           frequency))


class FeedbackResults():
    """
    This class stores the impedance results for a single step in the protocol.
    """
    class_version = str(Version(0, 4))

    def __init__(self, voltage, frequency, dt_ms, V_hv, hv_resistor, V_fb,
                 fb_resistor, calibration):

        self.voltage = voltage
        self.frequency = frequency
        self.V_hv = V_hv
        self.hv_resistor = hv_resistor
        self.V_fb = V_fb
        self.fb_resistor = fb_resistor
        self.time = np.arange(0, len(V_hv)) * dt_ms
        self.calibration = calibration
        self.version = self.class_version
        self._sanitize_data()

    def _sanitize_data(self):
        self.V_hv[self.hv_resistor < 0] = np.nan
        self.V_fb[self.fb_resistor < 0] = np.nan

    def _upgrade(self):
        """
        Upgrade the serialized object if necessary.

        Raises:
            FutureVersionError: file was written by a future version of the
                software.
        """
        logging.debug('[FeedbackResults]._upgrade()')
        if hasattr(self, 'version'):
            version = Version.fromstring(self.version)
        else:
            version = Version(0)
        logging.debug('[FeedbackResults] version=%s, class_version=%s' %
                      (str(version), self.class_version))
        if version > Version.fromstring(self.class_version):
            logging.debug('[FeedbackResults] version>class_version')
            raise FutureVersionError(Version.fromstring(self.class_version),
                                     version)
        elif version < Version.fromstring(self.class_version):
            if version < Version(0, 1):
                self.calibration = FeedbackCalibration()
            if version < Version(0, 2):
                # flag invalid data points
                self.version = str(Version(0, 2))
                self.fb_resistor[self.V_fb > 5] = -1
                self.hv_resistor[self.V_hv > 5] = -1
            if version < Version(0, 3):
                self.attempt = 0
                self.version = str(Version(0,    3))
                logging.info('[FeedbackResults] upgrade to version %s' %
                             self.version)
            if version < Version(0, 4):
                self.dt_ms = self.sampling_time_ms + \
                    self.delay_between_samples_ms
                del self.sampling_time_ms
                del self.delay_between_samples_ms
                self.voltage = self.options.voltage
                del self.options
                del self.attempt
                self.version = str(Version(0,    4))
                logging.info('[FeedbackResults] upgrade to version %s' %
                             self.version)
        else:
            # Else the versions are equal and don't need to be upgraded.
            pass

    def __setstate__(self, state):
        # convert lists to numpy arrays
        self.__dict__ = state
        for k, v in self.__dict__.items():
            if isinstance(v, list):
                self.__dict__[k] = np.array(v)
        self._upgrade()
        self._sanitize_data()

    def __getstate__(self):
        # convert numpy arrays/floats to standard lists/floats
        out = copy.deepcopy(self.__dict__)
        for k, v in out.items():
            if isinstance(v, np.ndarray):
                out[k] = v.tolist()
        return out

    def V_total(self):
        ind = mlab.find(self.hv_resistor >= 0)
        T = np.zeros(self.hv_resistor.shape)
        T[ind] = feedback_signal([self.calibration.C_hv[self.hv_resistor[ind]],
                                  self.calibration.R_hv
                                  [self.hv_resistor[ind]]],
                                 self.frequency, 10e6,
                                 self.calibration.hw_version)
        return self.V_hv / T

    def V_actuation(self):
        if self.calibration.hw_version.major == 1:
            return self.V_total() - np.array(self.V_fb)
        else:
            return self.V_total()

    def Z_device(self):
        ind = mlab.find(self.fb_resistor >= 0)
        R_fb = np.zeros(self.fb_resistor.shape)
        C_fb = np.zeros(self.fb_resistor.shape)
        R_fb[ind] = self.calibration.R_fb[self.fb_resistor[ind]]
        C_fb[ind] = self.calibration.C_fb[self.fb_resistor[ind]]
        if self.calibration.hw_version.major == 1:
            return (R_fb / np.sqrt(1 + np.square(R_fb * C_fb * self.frequency *
                                                 2 * math.pi)) *
                    (self.V_total() / self.V_fb - 1))
        else:
            return (R_fb / np.sqrt(1 + np.square(R_fb * C_fb * self.frequency *
                                                 2 * math.pi)) *
                    (self.V_total() / self.V_fb))

    def min_impedance(self):
        return min(self.Z_device())

    def capacitance(self):
        return 1.0 / (2 * math.pi * self.frequency * self.Z_device())

    def x_position(self, area):
        if self.calibration.C_drop:
            C_drop = self.calibration.C_drop
        else:
            C_drop = self.capacitance()[-1] / area
        if self.calibration.C_filler:
            C_filler = self.calibration.C_filler
        else:
            C_filler = 0
        return ((self.capacitance() / area - C_filler) /
                (C_drop - C_filler) * np.sqrt(area))

    def mean_velocity(self, area, ind=None, threshold=0.95):
        if ind is None:
            ind = range(len(self.time))
        t, dxdt = self.dxdt(area, ind, threshold)
        x = self.x_position(area)
        ind_stop = ind[mlab.find(dxdt == 0)[0]]
        dx = x[ind_stop] - x[ind[0]]
        if max(dxdt > 0) and ind_stop < len(t):
            dt = t[ind_stop] - t[ind[0]]
            return dx / dt
        else:
            return 0

    def dxdt(self, area, ind=None, threshold=0.95):
        if ind is None:
            ind = range(len(self.time))
        dt = np.diff(self.time[ind])
        t = self.time[ind][1:] - (self.time[1] - self.time[0]) / 2.0
        C = self.capacitance()[ind]
        dCdt = np.diff(C) / dt

        if self.calibration.C_drop:
            C_drop = self.calibration.C_drop * area
        else:
            C_drop = C[-1]
        if self.calibration.C_filler:
            C_filler = self.calibration.C_filler * area
        else:
            C_filler = 0

        # find the time when the capacitance exceeds the specified threshold
        # (e.g., the drop has stopped moving once it has passed 95% of it's
        # final value)
        ind_stop = mlab.find((C[1:] - C_filler) / (C[-1] - C_filler) >
                             threshold)
        if len(ind_stop):
            # set all remaining velocities to 0
            dCdt[ind_stop[0]:] = 0

        return t, dCdt / (C_drop - C_filler) * np.sqrt(area)


class FeedbackResultsSeries():
    """
    This class stores the impedance results for a series of measurements versus
    another independent variable.
    """
    class_version = str(Version(0, 0))

    def __init__(self, xlabel):
        self.data = []
        self.version = self.class_version
        self.xlabel = xlabel
        self.x = np.zeros(0)
        
    def add_data(self, x, feedback_results):
        self.x = np.concatenate((self.x, [x]))
        self.data.append(feedback_results)
    
    @property
    def frequency(self):
        return self._concatenate_data_from_member('frequency')

    @property
    def voltage(self):
        return self._concatenate_data_from_member('voltage')

    def V_total(self, *args, **kwargs):
        return self._concatenate_data_from_function('V_total', *args, **kwargs)

    def V_actuation(self, *args, **kwargs):
        return self._concatenate_data_from_function('V_actuation', *args, **kwargs)

    def Z_device(self, *args, **kwargs):
        return self._concatenate_data_from_function('Z_device', *args, **kwargs)

    def capacitance(self, *args, **kwargs):
        return self._concatenate_data_from_function('capacitance', *args, **kwargs)

    def x_position(self, *args, **kwargs):
        return self._concatenate_data_from_function('x_position', *args, **kwargs)

    def mean_velocity(self, *args, **kwargs):
        return self._concatenate_data_from_function('mean_velocity', *args, **kwargs)

    def dxdt(self, *args, **kwargs):
        return self._concatenate_data_from_function('dxdt', *args, **kwargs)

    def _concatenate_data_from_function(self, fn, *args, **kwargs):
        data = getattr(self.data[0], fn)(*args, **kwargs)
        result = np.array(data.reshape((1, len(data))))
        for row in self.data[1:]:
            data = getattr(row, fn)(*args, **kwargs)
            result = np.concatenate((result, data.reshape((1, len(data)))))
        return result

    def _concatenate_data_from_member(self, name):
        data = getattr(self.data[0], name)
        result = np.array([data])
        for row in self.data[1:]:
            data = getattr(row, name)
            result = np.concatenate((result, np.array([data])))
        return result


class FeedbackCalibration():
    class_version = str(Version(0, 2))

    def __init__(self, R_hv=None, C_hv=None, R_fb=None, C_fb=None, C_drop=None,
                 C_filler=None, hw_version=None):
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
        if C_drop:
            self.C_drop = C_drop
        else:
            self.C_drop = None
        if C_filler:
            self.C_filler = C_filler
        else:
            self.C_filler = None
        if hw_version:
            self.hw_version = hw_version
        else:
            self.hw_version = Version(1)
        self.version = self.class_version

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
            if k == 'R_hv' or k == 'C_hv' or k == 'R_fb' or k == 'C_fb':
                self.__dict__[k] = np.array(v)
        if 'version' not in self.__dict__:
            self.version = str(Version(0, 0))
        self._upgrade()

    def _upgrade(self):
        """
        Upgrade the serialized object if necessary.

        Raises:
            FutureVersionError: file was written by a future version of the
                software.
        """
        logging.debug("[FeedbackCalibration]._upgrade()")
        version = Version.fromstring(self.version)
        logging.debug('[FeedbackCalibration] version=%s, class_version=%s',
                      str(version), self.class_version)
        if version > Version.fromstring(self.class_version):
            logging.debug('[FeedbackCalibration] version>class_version')
            raise FutureVersionError(Version.fromstring(self.class_version),
                                     version)
        elif version < Version.fromstring(self.class_version):
            if version < Version(0, 1):
                self.C_filler = None
                self.C_drop = None
                self.version = str(Version(0, 1))
            if version < Version(0, 2):
                self.hw_version = Version(1)
                self.version = str(Version(0, 2))
                logging.info('[FeedbackCalibration] upgrade to version %s',
                             self.version)
        # else the versions are equal and don't need to be upgraded


@decorator.decorator
def safe_series_resistor_index_read(f, self, channel, resistor_index=None):
    '''
    This decorator checks the resistor-index from the current context _(i.e.,
    the result of `self.series_resistor_index`)_.  If the resistor-index
    specified by the `resistor_index` keyword argument is different than the
    current context value, the series-resistor-index is temporarily set to the
    value of `resistor_index` to execute the wrapped function before restoring
    back to the original value.
    '''
    if resistor_index is not None:
        original_resistor_index = self.series_resistor_index(channel)
        # Save state of resistor-index
        if resistor_index != original_resistor_index:
            self.set_series_resistor_index(channel, resistor_index)

    value = f(self, channel)

    if (resistor_index is not None and
            resistor_index != original_resistor_index):
        # Restore state of resistor-index
        self.set_series_resistor_index(channel, original_resistor_index)
    return value


@decorator.decorator
def safe_series_resistor_index_write(f, self, channel, value,
                                     resistor_index=None):
    '''
    This decorator checks the resistor-index from the current context _(i.e.,
    the result of `self.series_resistor_index`)_.  If the resistor-index
    specified by the `resistor_index` keyword argument is different than the
    current context value, the series-resistor-index is temporarily set to the
    value of `resistor_index` to execute the wrapped function before restoring
    back to the original value.
    '''
    if resistor_index is not None:
        original_resistor_index = self.series_resistor_index(channel)
        # Save state of resistor-index
        if resistor_index != original_resistor_index:
            self.set_series_resistor_index(channel, resistor_index)

    value = f(self, channel, value)

    if (resistor_index is not None and
            resistor_index != original_resistor_index):
        # Restore state of resistor-index
        self.set_series_resistor_index(channel, original_resistor_index)
    return value


class DMFControlBoard(Base, SerialDevice):
    def __init__(self):
        Base.__init__(self)
        SerialDevice.__init__(self)
        self.calibration = None
        self.__aref__ = None

    @safe_series_resistor_index_read
    def series_capacitance(self, channel, resistor_index=None):
        '''
        Return the current series capacitance value for the specified channel.

        If `resistor_index` is not specified, the resistor-index from the
        current context _(i.e., the result of `self.series_resistor_index`)_ is
        used.  Otherwise, the series-resistor is temporarily set to the value
        of `resistor_index` to read the capacitance before restoring back to
        the original value.  See definition of
        `safe_series_resistor_index_read` decorator.
        '''
        return self._series_capacitance(channel)

    @safe_series_resistor_index_read
    def series_resistance(self, channel, resistor_index=None):
        '''
        Return the current series resistance value for the specified channel.

        If `resistor_index` is not specified, the resistor-index from the
        current context _(i.e., the result of `self.series_resistor_index`)_ is
        used.  Otherwise, the series-resistor is temporarily set to the value
        of `resistor_index` to read the resistance before restoring back to the
        original value.  See definition of `safe_series_resistor_index_read`
        decorator.
        '''
        return self._series_resistance(channel)

    @safe_series_resistor_index_write
    def set_series_capacitance(self, channel, value, resistor_index=None):
        '''
        Set the current series capacitance value for the specified channel.

        If `resistor_index` is not specified, the resistor-index from the
        current context _(i.e., the result of `self.series_resistor_index`)_ is
        used.  Otherwise, the series-resistor is temporarily set to the value
        of `resistor_index` to set the capacitance before restoring back to
        the original value.  See definition of
        `safe_series_resistor_index_write` decorator.
        '''
        return self._set_series_capacitance(channel, value)

    @safe_series_resistor_index_write
    def set_series_resistance(self, channel, value, resistor_index=None):
        '''
        Set the current series resistance value for the specified channel.

        If `resistor_index` is not specified, the resistor-index from the
        current context _(i.e., the result of `self.series_resistor_index`)_ is
        used.  Otherwise, the series-resistor is temporarily set to the value
        of `resistor_index` to set the resistance before restoring back to the
        original value.  See definition of `safe_series_resistor_index_write`
        decorator.
        '''
        return self._set_series_resistance(channel, value)

    @property
    def config_version(self):
        return unpack('HHH',
                      pack(6 * 'B',
                           *[self.persistent_read(self
                                                  .PERSISTENT_CONFIG_SETTINGS +
                                                  i) for i in range(2 * 3)]))

    def connect(self, port=None, baud_rate=115200):
        if port:
            logger.info("Try connecting to port %s..." % port)
            Base.connect(self, port, baud_rate)
            self.port = port
        else:
            self.get_port(baud_rate)
        logger.info("Poll control board for series resistors and "
                    "capacitance values.")

        R_hv = []
        C_hv = []
        R_fb = []
        C_fb = []
        try:
            i = 0
            while True:
                self.set_series_resistor_index(0, i)
                R_hv.append(self.series_resistance(0))
                C_hv.append(self.series_capacitance(0))
                i += 1
        except:
            logger.info("HV series resistors =% s" % R_hv)
            logger.info("HV series capacitance =% s" % C_hv)
        try:
            i = 0
            while True:
                self.set_series_resistor_index(1, i)
                R_fb.append(self.series_resistance(1))
                C_fb.append(self.series_capacitance(1))
                i += 1
        except:
            logger.info("Feedback series resistors=%s" % R_fb)
            logger.info("Feedback series capacitance=%s" % C_fb)
        self.calibration = FeedbackCalibration(R_hv, C_hv, R_fb, C_fb,
                                               hw_version=
                                               Version.fromstring
                                               (self
                                                .hardware_version()))
        self.set_series_resistor_index(0, 0)
        self.set_series_resistor_index(1, 0)
        self.__aref__ = self._aref()
        return self.RETURN_OK

    @property
    def baud_rate(self):
        data = np.zeros(4)
        for i in range(0, 4):
            data[i] = self.persistent_read(self.PERSISTENT_BAUD_RATE_ADDRESS +
                                           i)
        return unpack('L', pack('BBBB', *data))[0]

    @baud_rate.setter
    def baud_rate(self, rate):
        data = unpack('BBBB', pack('L', rate))
        for i in range(0, 4):
            self.persistent_write(self.PERSISTENT_BAUD_RATE_ADDRESS + i,
                                  data[i])
        self.__baud_rate = rate

    @property
    def serial_number(self):
        data = np.zeros(4)
        for i in range(0, 4):
            data[i] = self.persistent_read(
                self.PERSISTENT_SERIAL_NUMBER_ADDRESS + i)
        return unpack('L', pack('BBBB', *data))[0]

    @serial_number.setter
    def serial_number(self, number):
        data = unpack('BBBB', pack('L', number))
        for i in range(0, 4):
            self.persistent_write(self.PERSISTENT_SERIAL_NUMBER_ADDRESS + i,
                                  data[i])
        self.__serial_number = number

    @property
    def voltage_tolerance(self):
        data = np.zeros(4)
        for i in range(0, 4):
            data[i] = self.persistent_read(self.PERSISTENT_VOLTAGE_TOLERANCE +
                                           i)
        return unpack('f', pack('BBBB', *data))[0]

    @voltage_tolerance.setter
    def voltage_tolerance(self, tolerance):
        data = unpack('BBBB', pack('f', tolerance))
        for i in range(0, 4):
            self.persistent_write(self.PERSISTENT_VOLTAGE_TOLERANCE + i,
                                  data[i])
        self.__voltage_tolerance = tolerance

    @property
    def state_of_all_channels(self):
        return np.array(Base.state_of_all_channels(self))

    def set_state_of_all_channels(self, state):
        self.state_of_all_channels = state

    @state_of_all_channels.setter
    def state_of_all_channels(self, state):
        state_ = uint8_tVector()
        for i in range(0, len(state)):
            state_.append(int(state[i]))
        Base.set_state_of_all_channels(self, state_)

    @property
    def default_pin_modes(self):
        pin_modes = []
        for i in range(0, 53 / 8 + 1):
            mode = self.persistent_read(self.PERSISTENT_PIN_MODE_ADDRESS + i)
            for j in range(0, 8):
                if i * 8 + j <= 53:
                    pin_modes.append(~mode >> j & 0x01)
        return pin_modes

    def set_default_pin_modes(self, pin_modes):
        self.default_pin_modes = pin_modes

    @default_pin_modes.setter
    def default_pin_modes(self, pin_modes):
        for i in range(0, 53 / 8 + 1):
            mode = 0
            for j in range(0, 8):
                if i * 8 + j <= 53:
                    mode += pin_modes[i * 8 + j] << j
            self.persistent_write(self.PERSISTENT_PIN_MODE_ADDRESS + i, ~mode &
                                  0xFF)

    @property
    def default_pin_states(self):
        pin_states = []
        for i in range(0, 53 / 8 + 1):
            state = self.persistent_read(self.PERSISTENT_PIN_STATE_ADDRESS + i)
            for j in range(0, 8):
                if i * 8 + j <= 53:
                    pin_states.append(~state >> j & 0x01)
        return pin_states

    def set_default_pin_states(self, pin_states):
        self.default_pin_states = pin_states

    @default_pin_states.setter
    def default_pin_states(self, pin_states):
        for i in range(0, 53 / 8 + 1):
            state = 0
            for j in range(0, 8):
                if i * 8 + j <= 53:
                    state += pin_states[i * 8 + j] << j
            self.persistent_write(self.PERSISTENT_PIN_STATE_ADDRESS + i, ~state
                                  & 0xFF)

    def analog_reads(self, pins, n_samples):
        pins_ = uint8_tVector()
        if hasattr(pins, '__len__'):
            for i in range(0, len(pins)):
                pins_.append(pins[i])
        else:
            pins_.append(pins)
        return np.array(Base.analog_reads(self, pins_, n_samples))

    def measure_impedance_non_blocking(self,
                                       sampling_window_ms,
                                       n_sampling_windows,
                                       delay_between_windows_ms,
                                       interleave_samples,
                                       rms,
                                       state):
        state_ = uint8_tVector()
        for i in range(0, len(state)):
            state_.append(int(state[i]))
        Base.measure_impedance_non_blocking(self,
                                            sampling_window_ms,
                                            n_sampling_windows,
                                            delay_between_windows_ms,
                                            interleave_samples,
                                            rms,
                                            state_)

    def impedance_buffer_to_feedback_result(self, buffer):
        dt_ms = buffer[-1]
        buffer = buffer[:-1]
        V_hv = buffer[0::4] / (64*1023.0) * self.__aref__ / 2.0 / np.sqrt(2)
        hv_resistor = buffer[1::4].astype(int)
        V_fb = buffer[2::4] / (64*1023.0) * self.__aref__ / 2.0 / np.sqrt(2)
        fb_resistor = buffer[3::4].astype(int)
        voltage = self.waveform_voltage()
        frequency = self.waveform_frequency()
        return FeedbackResults(voltage, frequency, dt_ms, V_hv, hv_resistor,
                               V_fb, fb_resistor, self.calibration)

    def get_impedance_data(self):
        buffer = np.array(Base.get_impedance_data(self))
        return self.impedance_buffer_to_feedback_result(buffer)

    def measure_impedance(self,
                          sampling_window_ms,
                          n_sampling_windows,
                          delay_between_windows_ms,
                          interleave_samples,
                          rms,
                          state):
        state_ = uint8_tVector()
        for i in range(0, len(state)):
            state_.append(int(state[i]))

        buffer = np.array(Base.measure_impedance(self,
                                                 sampling_window_ms,
                                                 n_sampling_windows,
                                                 delay_between_windows_ms,
                                                 interleave_samples,
                                                 rms,
                                                 state_))
        return self.impedance_buffer_to_feedback_result(buffer)   
        
    def i2c_write(self, address, data):
        data_ = uint8_tVector()
        for i in range(0, len(data)):
            data_.append(int(data[i]))
        Base.i2c_write(self, address, data_)

    def i2c_read(self, address, n_bytes_to_read):
        return np.array(Base.i2c_read(self, address, n_bytes_to_read))

    def i2c_send_command(self, address, cmd, data, delay_ms=100):
        data_ = uint8_tVector()
        for i in range(0, len(data)):
            data_.append(int(data[i]))
        return np.array(Base.i2c_send_command(self, address, cmd, data_,
                                              delay_ms))

    def test_connection(self, port, baud_rate):
        try:
            if self.connect(port, baud_rate) == self.RETURN_OK:
                return True
        except Exception, why:
            logger.info('On port %s, %s' % (port, why))
        return False

    def flash_firmware(self, hardware_version):
        logger.info("[DMFControlBoard].flash_firmware()")
        reconnect = self.connected()
        if reconnect:
            self.disconnect()
        try:
            hex_path = package_path().joinpath('firmware', 'mega2560', '%s_%s'
                                               % (hardware_version.major,
                                                  hardware_version.minor),
                                               'dmf_control_board.hex')
            logger.info("hex_path=%s" % hex_path)

            logger.info("initializing avrdude")
            protocol = 'wiring'
            baud_rate = 115200
            microcontroller = 'mega2560'
            avrdude = AvrDude(protocol, microcontroller, baud_rate,
                              port=self.port)

            logger.info("flashing firmware: hardware version %s"
                        % (hardware_version))
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

    @property
    def PERSISTENT_AREF_ADDRESS(self):
        hardware_version = self.hardware_version()
        if (hardware_version == '1.0' or hardware_version == '1.1' or
                hardware_version == '1.2'):
            return self.PERSISTENT_CONFIG_SETTINGS + 6
        else:
            raise PersistentSettingDoesNotExist()

    @property
    def PERSISTENT_SWITCHING_BOARD_I2C_ADDRESS(self):
        hardware_version = self.hardware_version()
        if (hardware_version == '1.0' or hardware_version == '1.1' or
                hardware_version == '1.2'):
            return self.PERSISTENT_CONFIG_SETTINGS + 7
        else:
            return self.PERSISTENT_CONFIG_SETTINGS + 6

    @property
    def PERSISTENT_WAVEOUT_GAIN_1_ADDRESS(self):
        hardware_version = self.hardware_version()
        if (hardware_version == '1.0' or hardware_version == '1.1' or
                hardware_version == '1.2'):
            return self.PERSISTENT_CONFIG_SETTINGS + 8
        elif hardware_version == '1.3':
            return self.PERSISTENT_CONFIG_SETTINGS + 7
        else:
            raise PersistentSettingDoesNotExist()

    @property
    def PERSISTENT_VGND_ADDRESS(self):
        hardware_version = self.hardware_version()
        if (hardware_version == '1.0' or hardware_version == '1.1' or
                hardware_version == '1.2'):
            return self.PERSISTENT_CONFIG_SETTINGS + 9
        elif hardware_version == '1.3':
            return self.PERSISTENT_CONFIG_SETTINGS + 8
        else:
            raise PersistentSettingDoesNotExist()

    @property
    def PERSISTENT_SIGNAL_GENERATOR_BOARD_I2C_ADDRESS(self):
        hardware_version = self.hardware_version()
        if hardware_version >= '2.0':
            return self.PERSISTENT_CONFIG_SETTINGS + 7
        else:
            raise PersistentSettingDoesNotExist()

    @property
    def PERSISTENT_VOLTAGE_TOLERANCE(self):
        hardware_version = self.hardware_version()
        if (hardware_version == '1.0' or hardware_version == '1.1' or
                hardware_version == '1.2'):
            return self.PERSISTENT_CONFIG_SETTINGS + 62
        elif hardware_version == '1.3':
            return self.PERSISTENT_CONFIG_SETTINGS + 61
        else:  # hardware_version >= 2.0
            return self.PERSISTENT_CONFIG_SETTINGS + 76

    @property
    def auto_adjust_amplifier_gain(self):
        return self._auto_adjust_amplifier_gain()

    @auto_adjust_amplifier_gain.setter
    def auto_adjust_amplifier_gain(self, value):
        return self._set_auto_adjust_amplifier_gain(value)
    @property
    def amplifier_gain(self):
        return self._amplifier_gain()

    @amplifier_gain.setter
    def amplifier_gain(self, value):
        return self._set_amplifier_gain(value)

    @property
    def aref(self):
        return self.persistent_read(self.PERSISTENT_AREF_ADDRESS)

    @aref.setter
    def aref(self, value):
        return self.persistent_write(self.PERSISTENT_AREF_ADDRESS, value)

    @property
    def switching_board_i2c_address(self):
        return self.persistent_read(self
                                    .PERSISTENT_SWITCHING_BOARD_I2C_ADDRESS)

    @switching_board_i2c_address.setter
    def switching_board_i2c_address(self, value):
        return self.persistent_write(self
                                     .PERSISTENT_SWITCHING_BOARD_I2C_ADDRESS,
                                     value)

    @property
    def waveout_gain_1(self):
        return self.persistent_read(self.PERSISTENT_WAVEOUT_GAIN_1_ADDRESS)

    @waveout_gain_1.setter
    def waveout_gain_1(self, value):
        return self.persistent_write(self.PERSISTENT_WAVEOUT_GAIN_1_ADDRESS,
                                     value)

    @property
    def vgnd(self):
        return self.persistent_read(self.PERSISTENT_VGND_ADDRESS)

    @vgnd.setter
    def vgnd(self, value):
        return self.persistent_write(self.PERSISTENT_VGND_ADDRESS, value)

    @property
    def signal_generator_board_i2c_address(self):
        return self.persistent_read(
            self.PERSISTENT_SIGNAL_GENERATOR_BOARD_I2C_ADDRESS)

    @signal_generator_board_i2c_address.setter
    def signal_generator_board_i2c_address(self, value):
        return self.persistent_write(
            self.PERSISTENT_SIGNAL_GENERATOR_BOARD_I2C_ADDRESS, value)

    def read_all_series_channel_values(self, f, channel):
        '''
        Return all values for the specified channel of the type corresponding
        to the function `f`, where `f` is either `self.series_resistance` or
        `self.series_capacitance`.
        '''
        values = []
        channel_max_param_count = [3, 5]
        for i in range(channel_max_param_count[channel]):
            try:
                values.append(f(channel, i))
            except RuntimeError:
                break
        return values

    def write_all_series_channel_values(self, read_f, write_f, channel,
                                        values):
        '''
        Return all values for the specified channel of the type corresponding
        to the function `f`, where `f` is either `self.series_resistance` or
        `self.series_capacitance`.
        '''
        # Read the current values, and only update the values that are
        # different.
        original_values = self.read_all_series_channel_values(read_f, channel)

        # Make sure that the number of supplied values matches the number of
        # corresponding values read from the channel.
        assert(len(values) == len(original_values))

        for i in range(len(original_values)):
            if values[i] != original_values[i]:
                write_f(channel, values[i], i)

    @property
    def a0_series_resistance(self):
        return self.read_all_series_channel_values(self.series_resistance, 0)

    @a0_series_resistance.setter
    def a0_series_resistance(self, values):
        return self.write_all_series_channel_values(self.series_resistance,
                                                    self.set_series_resistance,
                                                    0, values)

    @property
    def a0_series_capacitance(self):
        return self.read_all_series_channel_values(self.series_capacitance, 0)

    @a0_series_capacitance.setter
    def a0_series_capacitance(self, values):
        return self.write_all_series_channel_values(self.series_capacitance,
                                                    self
                                                    .set_series_capacitance,
                                                    0, values)

    @property
    def a1_series_resistance(self):
        return self.read_all_series_channel_values(self.series_resistance, 1)

    @a1_series_resistance.setter
    def a1_series_resistance(self, values):
        return self.write_all_series_channel_values(self.series_resistance,
                                                    self.set_series_resistance,
                                                    1, values)

    @property
    def a1_series_capacitance(self):
        return self.read_all_series_channel_values(self.series_capacitance, 1)

    @a1_series_capacitance.setter
    def a1_series_capacitance(self, values):
        return self.write_all_series_channel_values(self.series_capacitance,
                                                    self
                                                    .set_series_capacitance,
                                                    1, values)

    @property
    def config_attribute_names(self):
        return ['aref', 'waveout_gain_1',
                'vgnd', 'a0_series_resistance', 'a0_series_capacitance',
                'a1_series_resistance', 'a1_series_capacitance',
                'signal_generator_board_i2c_address',
                'amplifier_gain', 'switching_board_i2c_address',
                'voltage_tolerance', ]

    def read_config(self):
        '''
        '''
        except_types = (PersistentSettingDoesNotExist, )
        return OrderedDict([(a, safe_getattr(self, a, except_types))
                            for a in self.config_attribute_names])

    def write_config(self, config):
        device_config = self.read_config()
        common_keys = set(config.keys()).intersection(device_config.keys())
        for k in device_config.keys():
            if k in common_keys and (device_config[k] is not None and
                                     config[k] is not None):
                setattr(self, k, config[k])

    def debug_string(self):
        return "".join(map(chr, self.debug_buffer()))
