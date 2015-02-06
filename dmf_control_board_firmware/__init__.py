# coding: utf-8
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
import math
import warnings

import numpy as np
import pandas
from scipy.signal import savgol_filter
import matplotlib.mlab as mlab
from path_helpers import path
from microdrop_utility import Version, FutureVersionError
from base_node import BaseNode

from dmf_control_board_base import DMFControlBoard as Base
from dmf_control_board_base import uint8_tVector
# Import firmware constants
from dmf_control_board_base import INPUT, OUTPUT, HIGH, LOW
from serial_device import SerialDevice
from arduino_helpers.context import auto_context, Board, Uploader
from calibrate.feedback import compute_from_transfer_function

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

    Modules that need to compile against `dmf_control_boad` should use this
    function to locate the appropriate include directories.

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


class BadVGND(Exception):
    pass


class FeedbackResults():
    """
    This class stores the impedance results for a single step in the protocol.
    """
    class_version = str(Version(0, 6))

    def __init__(self, voltage, frequency, dt_ms, V_hv, hv_resistor, V_fb,
                 fb_resistor, calibration, area=0, amplifier_gain=None,
                 vgnd_hv=None, vgnd_fb=None):
        self.voltage = voltage
        self.frequency = frequency
        self.V_hv = V_hv
        self.hv_resistor = hv_resistor
        self.V_fb = V_fb
        self.fb_resistor = fb_resistor
        self.time = np.arange(0, len(V_hv)) * dt_ms
        self.calibration = calibration
        self.version = self.class_version
        self.area = area
        self.amplifier_gain = amplifier_gain
        self.vgnd_hv = vgnd_hv
        self.vgnd_fb = vgnd_fb
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
            if version < Version(0, 4):
                del self.sampling_time_ms
                del self.delay_between_samples_ms
                self.voltage = self.options.voltage
                del self.options
                del self.attempt
            if version < Version(0, 5):
                self.area = 0
                self.version = str(Version(0, 5))
            if version < Version(0, 6):
                self.amplifier_gain = None
                self.vgnd_hv = None
                self.vgnd_fb = None
                self.version = str(Version(0, 6))
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
        '''
        Compute the input voltage _(i.e., `V1`)_ based on the measured
        high-voltage feedback values for `V2`, using the high-voltage transfer
        function.
        '''
        ind = mlab.find(self.hv_resistor >= 0)
        V1 = np.empty(self.hv_resistor.shape)
        V1.fill(np.nan)
        V1[ind] = compute_from_transfer_function(self.calibration.hw_version
                                                 .major, 'V1',
                                                 V2=self.V_hv[ind], R1=10e6,
                                                 R2=self.calibration.R_hv
                                                 [self.hv_resistor[ind]],
                                                 C2=self.calibration.C_hv
                                                 [self.hv_resistor[ind]],
                                                 f=self.frequency)
        # convert to masked array
        return np.ma.masked_invalid(pandas.TimeSeries(V1,
            pandas.to_datetime(self.time, unit='s')
        ).interpolate(method='time').values)

    def V_actuation(self):
        '''
        Return the voltage drop across the device _(i.e., the `Z1` load)_ for
        each feedback measurement.

        Consider the feedback circuit diagrams below for the feedback
        measurement circuits of the two the control board hardware versions.

                         # Hardware V1 #          # Hardware V2 #

                         V_1 @ frequency          V_1 @ frequency
                        ┬    ┯                        ┯
                        │  ┌─┴─┐                    ┌─┴─┐    ┌───┐
            V_actuation │  │Z_1│                    │Z_1│  ┌─┤Z_2├─┐
                        │  └─┬─┘                    └─┬─┘  │ └───┘ │
                        ┴    ├───⊸ V_2                │    │  │╲   ├───⊸ V_2
                           ┌─┴─┐                      └────┴──│-╲__│
                           │Z_2│                           ┌──│+╱
                           └─┬─┘                           │  │╱
                            ═╧═                            │
                             ¯                            ═╧═
                                                           ¯

        Note that in the case of hardware version 1, the input voltage `V1` is
        divided across `Z1` and the feedback measurement load `Z2`.  Therefore,
        the effective _actuation_ voltage across the DMF device is less than
        `V1`.  Specifically, the effective _actuation_ voltage is `V1 - V2`.

        In hardware version 2, since the positive terminal of
        the op-amp is attached to _(virtual)_ ground, the negative op-amp
        terminal is also at ground potential.  It follows that the actuation
        voltage is equal to `V1` on hardware version 2.
        '''
        if self.calibration.hw_version.major == 1:
            return self.V_total() - np.array(self.V_fb)
        else:
            return self.V_total()

    def Z_device(self, filter_order=None, window_size=None, tol=0.05):
        '''
        Compute the impedance _(including resistive and capacitive load)_ of
        the DMF device _(i.e., dielectric and droplet)_.

        See `dmf_control_board.calibrate.compute_from_transfer_function`
        docstring for details.
        '''
        ind = mlab.find(self.fb_resistor >= 0)
        Z1 = np.empty(self.fb_resistor.shape)
        Z1.fill(np.nan)

        R2 = self.calibration.R_fb[self.fb_resistor[ind]]
        C2 = self.calibration.C_fb[self.fb_resistor[ind]]
        Z1[ind] = compute_from_transfer_function(self.calibration.hw_version
                                                 .major, 'Z1',
                                                 V1=self.V_total()[ind],
                                                 V2=self.V_fb[ind], R2=R2,
                                                 C2=C2, f=self.frequency)
        # convert to masked array
        Z1 = np.ma.masked_invalid(pandas.TimeSeries(Z1,
            pandas.to_datetime(self.time, unit='s')
        ).interpolate(method='time').values)

        # if we're filtering and we don't have a window size specified,
        # automatically determine one
        if filter_order and window_size is None:
            window_size = self._get_window_size(tol)

        # if the filter_order is None or if the window size is smaller than
        # filter_order + 2, don't filter
        if filter_order is None or \
            (window_size and window_size < filter_order + 2):
            pass
        else:
            # if the window size is less than half the sample length
            if window_size and window_size < len(Z1) / 2:
                # suppress polyfit warnings
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    Z1 = savgol_filter(Z1, window_size, filter_order)
            else: # fit a line
                result = self.mean_velocity(tol=tol)
                if result['dt'] and \
                    result['dt'] > 0.1 * self.time[-1] and result['p'][0] > 0:
                    if self.calibration._C_drop:
                        C_drop = self.calibration.C_drop(self.frequency)
                    else:
                        C_drop = self.capacitance()[-1] / self.area
                    if self.calibration._C_filler:
                        C_filler = self.calibration.C_filler(self.frequency)
                    else:
                        C_filler = 0
                    x = result['p'][0]*self.time + result['p'][1]
                    C = self.area * (x * (C_drop - C_filler) / \
                                     np.sqrt(self.area) + C_filler)
                    Z1 = 1.0 / (2.0 * math.pi * self.frequency * C)
                    Z1[mlab.find(self.time==result['t_end'])[0]+1:] = \
                        Z1[mlab.find(self.time==result['t_end'])[0]]
                else:
                    Z1 = np.mean(Z1)*np.ones(Z1.shape)
        return Z1

    def min_impedance(self):
        return min(self.Z_device())

    def capacitance(self, filter_order=None, window_size=None, tol=0.05):
        '''
        Compute the capacitance of the DMF device _(i.e., dielectric and
        droplet)_ based on the computed impedance value.

        Note: this assumes impedance is purely capacitive load.
        TODO: Is this assumption ok?
        '''
        C = 1.0 / (2.0 * math.pi * self.frequency *
                   self.Z_device(filter_order=filter_order,
                                 window_size=window_size, tol=tol))
        return np.ma.masked_invalid(
            pandas.TimeSeries(
                C, pandas.to_datetime(self.time, unit='s')
            ).interpolate(method='time', downcast=None).values
        )

    def x_position(self, filter_order=None, window_size=None, tol=0.05):
        '''
        Calculate $x$-position according to:

               __ | C      |
                           ╲╱ a   ⋅ | - - Cf |
                  | a      |
        x = ──────────────
                Cd - Cf

        where:

         - $C$ is the measured capacitance.
         - $c_f$ is the capacitance of the filler medium per unit area
           _(e.g., air)_.
         - $c_d$ is the capacitance of an electrode completely covered in
           liquid per unit area.
         - $a$ is the area of the actuated electrode(s).

        Note that this equation for $x$ assumes a single drop moving across a
        square electrode.
        '''
        if self.calibration._C_drop:
            C_drop = self.calibration.C_drop(self.frequency)
        else:
            C_drop = self.capacitance()[-1] / self.area
        if self.calibration._C_filler:
            C_filler = self.calibration.C_filler(self.frequency)
        else:
            C_filler = 0

        return (self.capacitance(filter_order=filter_order,
                                 window_size=window_size, tol=tol) / self.area
                - C_filler) / (C_drop - C_filler) * np.sqrt(self.area)

    def mean_velocity(self, tol=0.05):
        '''
        Calculate the mean velocity for a step (mm/ms which is equivalent to
        m/s). Fit a line to the capacitance data and get the slope.
        '''
        dx = None
        dt = None
        p = None
        ind = None
        t_end = None

        if self.area == 0:
            return dict(dx=dx, dt=dt, p=p, ind=ind, t_end=t_end)

        x = self.x_position()

        # find the first and last valid indices
        ind_start = mlab.find(x.mask==False)[0]
        ind_last = mlab.find(x.mask==False)[-1]

        # if the original x value is within tol % of the final x value, include
        # all samples
        if x[ind_start] > (1 - tol) * x[ind_last] or x[ind_last] < 0:
            ind_stop = ind_last
        else: # otherwise, stop when x reaches (1 - tol) % of it's final value
            ind_stop = mlab.find(x > (1 - tol) * x[ind_last])[0]

        ind = [ind_start, ind_stop]

        # if we have at least 2 valid samples
        if len(ind) >=2:
            dx = np.diff(x[ind])[0]
            dt = np.diff(self.time[ind])[0] # ms

            # suppress polyfit warnings
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                # fit a line to the data
                p = np.polyfit(self.time[ind[0]:ind[1]], x[ind[0]:ind[1]], 1)

            # find time when the the line intercepts x[ind_last]
            ind_stop = mlab.find(self.time > \
                                 (x[ind_last] - p[1]) / p[0])
            if len(ind_stop):
                t_end = self.time[ind_stop[0]]
            else:
                t_end = self.time[-1]
        return dict(dx=dx, dt=dt, p=p, ind=ind, t_end=t_end)

    def _get_window_size(self, tol=0.05):
        dt = self.time[1]-self.time[0]
        # calculate the mean velocity
        result = self.mean_velocity(tol=tol)
        window_size = None

        # calculate a default filtering window size
        if result['dt'] and \
            result['dt'] > 0.1 * self.time[-1] and result['p'][0] > 0:

            # get the velocity from the slope of the fit to the capacitance data
            mean_dxdt = result['p'][0]

            # pick a window size that corresponds to the time it takes to cover
            # half the electrode (must be an odd number)
            window_size = np.round(0.5 / (mean_dxdt / np.sqrt(self.area)) / \
                                   dt / 2.0) * 2.0 + 1
        return window_size

    def dxdt(self, filter_order=None, window_size=None, tol=0.05):
        x = self.x_position()

        if filter_order and window_size is None:
            window_size = self._get_window_size(tol)

        dt = self.time[1]-self.time[0]

        # if the filter_order is None or if the window size is smaller than
        # filter_order + 2, don't filter
        if filter_order is None or \
            (window_size and window_size < filter_order + 2):
            dx = np.diff(x)
            t = self.time[1:] - dt/2.0
        else: # filter
            t = self.time
            dx = np.zeros(t.shape)
            if window_size:
                # if the window size is less than half the sample length
                if window_size < len(x) / 2:
                    # suppress polyfit warnings
                    with warnings.catch_warnings():
                        warnings.simplefilter("ignore")
                        dx = savgol_filter(x, window_size, filter_order, 1)
                else: # use the average velocity
                    result = self.mean_velocity(tol=tol)
                    mean_dxdt = 0
                    if result['p'] is not None:
                        mean_dxdt = result['p'][0]
                    dx[:mlab.find(t==result['t_end'])[0]+1] = mean_dxdt * dt
            else:
                # otherwise, leave dx = 0
                pass
        return t, dx / dt


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
        return np.ma.masked_invalid(result)

    def _concatenate_data_from_member(self, name):
        data = getattr(self.data[0], name)
        result = np.array([data])
        for row in self.data[1:]:
            data = getattr(row, name)
            result = np.concatenate((result, np.array([data])))
        return np.ma.masked_invalid(result)


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
            self._C_drop = C_drop
        else:
            self._C_drop = None
        if C_filler:
            self._C_filler = C_filler
        else:
            self._C_filler = None
        if hw_version:
            self.hw_version = hw_version
        else:
            self.hw_version = Version(1)
        self.version = self.class_version

    def C_drop(self, frequency):
        try:
            return np.interp(frequency,
                             self._C_drop['frequency'],
                             self._C_drop['capacitance']
            )
        except:
            pass
        return self._C_drop

    def C_filler(self, frequency):
        try:
            return np.interp(frequency,
                             self._C_filler['frequency'],
                             self._C_filler['capacitance']
            )
        except:
            pass
        return self._C_filler

    def __getstate__(self):
        """Convert numpy arrays to lists for serialization"""
        out = copy.deepcopy(self.__dict__)
        for k, v in out.items():
            if isinstance(v, np.ndarray):
                out[k] = v.tolist()
        return out

    def __setstate__(self, state):
        """Convert lists to numpy arrays after loading serialized object"""
        # rename C_drop and C_filler members
        for k in ['C_drop', 'C_filler']:
            if k in state:
                state['_' + k] = state[k]
                del state[k]
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
                self._C_filler = None
                self._C_drop = None
                self.version = str(Version(0, 1))
            if version < Version(0, 2):
                self.hw_version = Version(1)
                self.version = str(Version(0, 2))
                logging.info('[FeedbackCalibration] upgrade to version %s',
                             self.version)
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
        if resistor_index is None:
            resistor_index = self.series_resistor_index(channel)
        value = self._series_capacitance(channel)
        try:
            if channel == 0:
                self.calibration.C_hv[resistor_index] = value
            else:
                self.calibration.C_fb[resistor_index] = value
        except:
            pass
        return value

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
        if resistor_index is None:
            resistor_index = self.series_resistor_index(channel)
        value = self._series_resistance(channel)
        try:
            if channel == 0:
                self.calibration.R_hv[resistor_index] = value
            else:
                self.calibration.R_fb[resistor_index] = value
        except:
            pass
        return value

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
        if resistor_index is None:
            resistor_index = self.series_resistor_index(channel)
        try:
            if channel == 0:
                self.calibration.C_hv[resistor_index] = value
            else:
                self.calibration.C_fb[resistor_index] = value
        except:
            pass
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
        if resistor_index is None:
            resistor_index = self.series_resistor_index(channel)
        try:
            if channel == 0:
                self.calibration.R_hv[resistor_index] = value
            else:
                self.calibration.R_fb[resistor_index] = value
        except:
            pass
        return self._set_series_resistance(channel, value)

    @property
    def config_version(self):
        return tuple(
            self.persistent_read_multibyte(self.PERSISTENT_CONFIG_SETTINGS,
                                           count=3, dtype=np.uint16))

    def connect(self, port=None, baud_rate=115200):
        if port:
            logger.info("Try connecting to port %s..." % port)
            Base.connect(self, port, baud_rate)
            self.port = port
        else:
            self.get_port(baud_rate)
            return self.RETURN_OK

        self._i2c_devices = {}

        name = self.name()
        version = self.hardware_version()
        firmware = self.software_version()
        serial_number_string = ""
        try:
            serial_number_string = ", S/N %03d" % self.serial_number
        except: # need to catch exceptions here because this call will generate
                # an error on old firmware which will prevent us from getting
                # the opportunity to apply a firmware update.
            pass
        logger.info("Connected to %s v%s (Firmware: %s%s)" % 
                    (name, version, firmware, serial_number_string))
        
        logger.info("Poll control board for series resistors and "
                    "capacitance values.")

        self._read_calibration_data()

        try:
            self.__aref__ = self._aref()
            logger.info("Analog reference = %.2f V" % self.__aref__)
        except: # need to catch exceptions here because this call will generate
                # an error on old firmware which will prevent us from getting
                # the opportunity to apply a firmware update.
            pass

        # Check VGND for both analog channels
        expected = 2**10/2
        v = {}
        channels = [0,1]
        damaged = []
        for channel in channels:
            try:
                v[channel] = np.mean(self.analog_reads(channel, 10))
                logger.info("A%d VGND = %.2f V (%.2f%% of Aref)" % (
                    channel, self.__aref__ * v[channel] / (2**10),
                    100.0 * v[channel] / (2**10)))
                # Make sure that the VGND is close to the expected value;
                # otherwise, the op-amp may be damaged (expected error
                # is <= 10%).
                if np.abs(v[channel] - expected) / expected > .1:
                    damaged.append(channel)
            except: # need to catch exceptions here because this call will
                    # generate an error on old firmware which will prevent us
                    # from getting the opportunity to apply a firmware update.
                break

        self._i2c_scan()

        if damaged:
            if len(damaged) == 1:
                msg = "Analog channel %d appears" % damaged[0]
            else:
                msg = "Analog channels %s appear" % damaged
            raise BadVGND(msg + " to be damaged. You may need to replace the "
                          "op-amp on the control board.")

        return self.RETURN_OK

    def _i2c_scan(self):
        logger.info("Scan i2c bus:")
        # scan for devices on the i2c bus
        try:
            for address in self.i2c_scan():
                try:
                    node = BaseNode(self, address)
                    description = ("%s v%s (Firmware v%s, S/N %03d)" % 
                        (node.name(), node.hardware_version(),
                         node.software_version(), node.serial_number)) 
                except:
                    description = "?" % address
                self._i2c_devices[address] = description
                logger.info("\t%d: %s" % (address, description))
        except: # need to catch exceptions here because this call will
                # generate an error on old firmware which will prevent us
                # from getting the opportunity to apply a firmware update.
            pass

    def _read_calibration_data(self):
        R_hv = self.a0_series_resistance
        C_hv = self.a0_series_capacitance
        R_fb = self.a1_series_resistance
        C_fb = self.a1_series_capacitance
        logger.info("R_hv=%s" % R_hv)
        logger.info("C_hv=%s" % C_hv)
        logger.info("R_fb=%s" % R_fb)
        logger.info("C_fb=%s" % C_fb)
        self.calibration = FeedbackCalibration(R_hv, C_hv, R_fb, C_fb,
                                               hw_version=
                                               Version.fromstring
                                               (self
                                                .hardware_version()))

    def persistent_write(self, address, byte, refresh_config=False):
        '''
        Write a single byte to an address in persistent memory.

        If refresh_config is True, load_config() is called afterward to
        refresh the configuration settings.
        '''
        self._persistent_write(address, byte)
        if refresh_config:
            self.load_config(False)

    def persistent_read_multibyte(self, address, count=None,
                                  dtype=np.uint8):
        nbytes = np.dtype(dtype).itemsize
        if count is not None:
            nbytes *= count

        # Read enough bytes starting at specified address to match the
        # requested number of the specified data type.
        data_bytes = np.array([self.persistent_read(address + i)
                               for i in xrange(nbytes)], dtype=np.uint8)

        # Cast byte array as array of specified data type.
        result = data_bytes.view(dtype)

        # If no count was specified, we return a scalar value rather than the
        # resultant array.
        if count is None:
            return result[0]
        return result

    def persistent_write_multibyte(self, address, data, refresh_config=False):
        '''
        Write multiple bytes to an address in persistent memory.

        If refresh_config is True, load_config() is called afterward to
        refresh the configuration settings.
        '''
        for i, byte in enumerate(data.view(np.uint8)):
            self.persistent_write(address + i, int(byte))
        if refresh_config:
            self.load_config(False)

    @property
    def baud_rate(self):
        return self.persistent_read_multibyte(
            self.PERSISTENT_BAUD_RATE_ADDRESS, dtype=np.uint32)

    @baud_rate.setter
    def baud_rate(self, value):
        self.persistent_write_multibyte(self.PERSISTENT_BAUD_RATE_ADDRESS,
                                        np.array([value], dtype=np.uint32),
                                        True)
        self.__baud_rate = value

    @property
    def serial_number(self):
        return self.persistent_read_multibyte(
            self.PERSISTENT_SERIAL_NUMBER_ADDRESS, dtype=np.uint32)

    @serial_number.setter
    def serial_number(self, value):
        self.persistent_write_multibyte(self.PERSISTENT_SERIAL_NUMBER_ADDRESS,
                                        np.array([value], dtype=np.uint32),
                                        True)
        self.__serial_number = value

    @property
    def voltage_tolerance(self):
        if not hasattr(self, '__voltage_tolerance'):
            self.__voltage_tolerance = self.persistent_read_multibyte(
                self.PERSISTENT_VOLTAGE_TOLERANCE, dtype=np.float32)
        return self.__voltage_tolerance

    @voltage_tolerance.setter
    def voltage_tolerance(self, value):
        self.persistent_write_multibyte(self.PERSISTENT_VOLTAGE_TOLERANCE,
                                        np.array([value], dtype=np.float32),
                                        True)
        self.__voltage_tolerance = value

    @property
    def use_antialiasing_filter(self):
        return self.persistent_read(self.PERSISTENT_USE_ANTIALIASING_FILTER)

    @use_antialiasing_filter.setter
    def use_antialiasing_filter(self, value):
        return self.persistent_write(self.PERSISTENT_USE_ANTIALIASING_FILTER,
                                     value, True)

    @property
    def min_waveform_frequency(self):
        if not hasattr(self, '__min_waveform_frequency'):
            self.__min_waveform_frequency = self.persistent_read_multibyte(
                self.PERSISTENT_MIN_WAVEFORM_FREQUENCY, dtype=np.float32)
        return self.__min_waveform_frequency

    @min_waveform_frequency.setter
    def min_waveform_frequency(self, value):
        self.persistent_write_multibyte(self.PERSISTENT_MIN_WAVEFORM_FREQUENCY,
                                        np.array([value], dtype=np.float32),
                                        True)
        self.__min_waveform_frequency = value

    @property
    def max_waveform_frequency(self):
        if not hasattr(self, '__max_waveform_frequency'):
            self.__max_waveform_frequency = self.persistent_read_multibyte(
                self.PERSISTENT_MAX_WAVEFORM_FREQUENCY, dtype=np.float32)
        return self.__max_waveform_frequency

    @max_waveform_frequency.setter
    def max_waveform_frequency(self, value):
        self.persistent_write_multibyte(self.PERSISTENT_MAX_WAVEFORM_FREQUENCY,
                                        np.array([value], dtype=np.float32),
                                        True)
        self.__max_waveform_frequency = value

    @property
    def max_waveform_voltage(self):
        if not hasattr(self, '__max_waveform_voltage'):
            self.__max_waveform_voltage = self.persistent_read_multibyte(
                self.PERSISTENT_MAX_WAVEFORM_VOLTAGE, dtype=np.float32)
        return self.__max_waveform_voltage

    @max_waveform_voltage.setter
    def max_waveform_voltage(self, value):
        self.persistent_write_multibyte(self.PERSISTENT_MAX_WAVEFORM_VOLTAGE,
                                        np.array([value], dtype=np.float32),
                                        True)
        self.__max_waveform_voltage = value

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
                                  0xFF, True)

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
                                  & 0xFF, True)

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
        amplifier_gain = buffer[-1]
        vgnd_hv = buffer[-2]
        vgnd_fb = buffer[-3]
        dt_ms = buffer[-4]
        buffer = buffer[:-4]
        V_hv = buffer[0::4] / (64*1023.0) * self.__aref__ / 2.0 / np.sqrt(2)
        hv_resistor = buffer[1::4].astype(int)
        V_fb = buffer[2::4] / (64*1023.0) * self.__aref__ / 2.0 / np.sqrt(2)
        fb_resistor = buffer[3::4].astype(int)
        voltage = self.waveform_voltage()
        frequency = self.waveform_frequency()
        return FeedbackResults(voltage, frequency, dt_ms, V_hv, hv_resistor,
                               V_fb, fb_resistor, self.calibration,
                               amplifier_gain=amplifier_gain,
                               vgnd_hv=vgnd_hv, vgnd_fb=vgnd_fb)

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
        '''
        Measure voltage across load of each of the following control board
        feedback circuits:

         - Reference _(i.e., attenuated high-voltage amplifier output)_.
         - Load _(i.e., voltage across DMF device)_.

        The measured voltage _(i.e., `V2`)_ can be used to compute the
        impedance of the measured load, the input voltage _(i.e., `V1`)_, etc.
        '''
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

    def i2c_scan(self):
        return np.array(Base.i2c_scan(self))

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
        logger.info("test_connection(%s, %d)" % (port, baud_rate))
        try:
            if self.connect(port, baud_rate) == self.RETURN_OK:
                return True
        except Exception, why:
            logger.info('On port %s, %s' % (port, why))
        return False

    def flash_firmware(self, hardware_version=None):
        logger.info("[DMFControlBoard].flash_firmware()")
        reconnect = self.connected()
        if reconnect:
            if not hardware_version:
                hardware_version = Version.fromstring(self.hardware_version())
            self.disconnect()
        try:
            hex_path = package_path().joinpath('firmware', 'mega2560', '%s_%s'
                                               % (hardware_version.major,
                                                  hardware_version.minor),
                                               'dmf_control_board.hex')
            logger.info("hex_path=%s" % hex_path)

            context = auto_context()
            board = Board(context, 'mega2560')
            uploader = Uploader(board)

            logger.info("flashing firmware: hardware version %s"
                        % (hardware_version))

            logger.info(uploader.upload(hex_path.abspath(), self.port))

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
    def PERSISTENT_USE_ANTIALIASING_FILTER(self):
        hardware_version = self.hardware_version()
        if (hardware_version == '1.0' or hardware_version == '1.1' or
                hardware_version == '1.2'):
            return self.PERSISTENT_CONFIG_SETTINGS + 66
        elif hardware_version == '1.3':
            return self.PERSISTENT_CONFIG_SETTINGS + 65
        else:  # hardware_version >= 2.0
            return self.PERSISTENT_CONFIG_SETTINGS + 80

    @property
    def PERSISTENT_MIN_WAVEFORM_FREQUENCY(self):
        hardware_version = self.hardware_version()
        if (hardware_version == '1.0' or hardware_version == '1.1' or
                hardware_version == '1.2'):
            return self.PERSISTENT_CONFIG_SETTINGS + 67
        elif hardware_version == '1.3':
            return self.PERSISTENT_CONFIG_SETTINGS + 66
        else:  # hardware_version >= 2.0
            return self.PERSISTENT_CONFIG_SETTINGS + 81

    @property
    def PERSISTENT_MAX_WAVEFORM_FREQUENCY(self):
        hardware_version = self.hardware_version()
        if (hardware_version == '1.0' or hardware_version == '1.1' or
                hardware_version == '1.2'):
            return self.PERSISTENT_CONFIG_SETTINGS + 71
        elif hardware_version == '1.3':
            return self.PERSISTENT_CONFIG_SETTINGS + 70
        else:  # hardware_version >= 2.0
            return self.PERSISTENT_CONFIG_SETTINGS + 85

    @property
    def PERSISTENT_MAX_WAVEFORM_VOLTAGE(self):
        hardware_version = self.hardware_version()
        if (hardware_version == '1.0' or hardware_version == '1.1' or
                hardware_version == '1.2'):
            return self.PERSISTENT_CONFIG_SETTINGS + 75
        elif hardware_version == '1.3':
            return self.PERSISTENT_CONFIG_SETTINGS + 74
        else:  # hardware_version >= 2.0
            return self.PERSISTENT_CONFIG_SETTINGS + 89

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
        return self.persistent_write(self.PERSISTENT_AREF_ADDRESS, value,
                                     True)

    @property
    def switching_board_i2c_address(self):
        return self.persistent_read(self
                                    .PERSISTENT_SWITCHING_BOARD_I2C_ADDRESS)

    @switching_board_i2c_address.setter
    def switching_board_i2c_address(self, value):
        return self.persistent_write(self
                                     .PERSISTENT_SWITCHING_BOARD_I2C_ADDRESS,
                                     value,
                                     True)

    @property
    def waveout_gain_1(self):
        return self.persistent_read(self.PERSISTENT_WAVEOUT_GAIN_1_ADDRESS)

    @waveout_gain_1.setter
    def waveout_gain_1(self, value):
        return self.persistent_write(self.PERSISTENT_WAVEOUT_GAIN_1_ADDRESS,
                                     value,
                                     True)

    @property
    def vgnd(self):
        return self.persistent_read(self.PERSISTENT_VGND_ADDRESS)

    @vgnd.setter
    def vgnd(self, value):
        return self.persistent_write(self.PERSISTENT_VGND_ADDRESS, value, True)

    @property
    def signal_generator_board_i2c_address(self):
        return self.persistent_read(
            self.PERSISTENT_SIGNAL_GENERATOR_BOARD_I2C_ADDRESS)

    @signal_generator_board_i2c_address.setter
    def signal_generator_board_i2c_address(self, value):
        return self.persistent_write(
            self.PERSISTENT_SIGNAL_GENERATOR_BOARD_I2C_ADDRESS, value, True)

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
        return ['aref', 'waveout_gain_1', 'vgnd', 'a0_series_resistance',
                'a0_series_capacitance', 'a1_series_resistance',
                'a1_series_capacitance', 'signal_generator_board_i2c_address',
                'auto_adjust_amplifier_gain', 'amplifier_gain',
                'switching_board_i2c_address', 'voltage_tolerance',
                'min_waveform_frequency', 'max_waveform_frequency',
                'max_waveform_voltage', 'use_antialiasing_filter']

    def reset_config_to_defaults(self):
        self._reset_config_to_defaults()
        self._read_calibration_data()

    def read_config(self):
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
