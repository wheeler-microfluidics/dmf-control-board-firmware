# coding: utf-8
import logging
import time
import cPickle as pickle

import pandas as pd
import gtk
import numpy as np
import matplotlib
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import scipy.optimize as optimize

from microdrop_utility.gui import text_entry_dialog
from microdrop_utility import Version, is_float


def process_hv_attenuator_calibration(control_board, results):
    hardware_version = Version.fromstring(control_board
                                          .hardware_version())
    input_voltage = np.array(results['input_voltage'])
    frequencies = np.array(results['frequencies'])
    hv_measurements = (np.array(results['hv_measurements']) / 1023.0 * 5 -
                       2.5)
    hv_rms = np.transpose(np.array([np.max(hv_measurements[:, j, :], 1) -
                                    np.min(hv_measurements[:, j, :], 1)
                                    for j in range(0, len(frequencies))]) /
                          2. / np.sqrt(2))

    voltages = np.array(results['voltages'])
    attenuation = hv_rms / voltages

    # p[0]=C, p[1]=R2
    R1 = 10e6
    f = lambda p, x, R1: np.abs(1 / (R1 / p[1] + 1 + R1 * 2 * np.pi * p[0]
                                     * complex(0, 1) * x))
    if hardware_version.major == 2:
        f = lambda p, x, R1: np.abs(1 / (R1 / p[1] + R1 * 2 * np.pi * p[0]
                                         * complex(0, 1) * x))
    e = lambda p, x, y, R1: f(p, x, R1) - y
    fit_params = []

    colors = ['b', 'r', 'g']
    legend = []
    for i in range(0, np.size(hv_rms, 0)):
        ind = mlab.find(hv_rms[i, :] > .1)
        p0 = [control_board.calibration.C_hv[i],
              control_board.calibration.R_hv[i]]
        plt.loglog(frequencies[ind], f(p0, frequencies[ind], R1),
                 colors[i] + '--')
        legend.append('R$_{%d}$ (previous fit)' % i)

        if 'voltages' in results:
            voltages = np.array(results['voltages'])
            T = attenuation[i, ind]
            p1, success = optimize.leastsq(e, p0, args=(frequencies[ind],
                                                        T, R1))
            fit_params.append(p1)
            plt.loglog(frequencies[ind], f(p1, frequencies[ind], R1),
                     colors[i] + '-')
            legend.append('R$_{%d}$ (new fit)' % i)

            plt.plot(frequencies, attenuation[i], colors[i] + 'o')
            legend.append('R$_{%d}$ (scope measurements)' % i)

            # update control board calibration
            control_board.set_series_resistor_index(0, i)
            control_board.set_series_resistance(0, abs(p1[1]))
            control_board.set_series_capacitance(0, abs(p1[0]))
            # reconnnect to update calibration data
            control_board.connect()
        else:  # Control board measurements
            fit_params.append(p0)
            plt.plot(frequencies, attenuation[i], colors[i] + 'o')
            legend.append('R$_{%d}$ (CB measurements)' % i)

    plt.legend(legend)
    a = plt.gca()
    a.set_xlabel('Frequency (Hz)')
    a.set_ylabel('Attenuation')
    a.set_title('HV attenuation')


def measure_board_hv(control_board, n_samples=10, sampling_ms=10,
                     delay_between_samples_ms=0):
    '''
    Read RMS voltage samples from control board high-voltage feedback circuit.
    '''
    results = control_board.measure_impedance(10, 10, 0, True, True, [])
    data = pd.DataFrame({'board measured V': results.V_hv})
    data['divider resistor index'] = results.hv_resistor
    return data


def find_good(control_board, actuation_steps, resistor_index, start_index,
              end_index):
    '''
    Use a binary search over the range of provided actuation_steps to find the
    maximum actuation voltage that is measured by the board feedback circuit
    using the specified feedback resistor.
    '''
    lower = start_index
    upper = end_index
    while lower < upper - 1:
        index = lower + (upper - lower) / 2
        v = actuation_steps[index]
        control_board.set_waveform_voltage(v)
        data = measure_board_hv(control_board)
        valid_data = data[data['divider resistor index'] >= 0]

        if (valid_data['divider resistor index'] <
                resistor_index).sum():
            # We have some measurements from another resistor.
            upper = index
        else:
            lower = index
    control_board.set_waveform_voltage(actuation_steps[lower])
    data = measure_board_hv(control_board)
    return lower, data


def get_oscope_reading():
    import PyZenity

    response = None
    while response is None:
        response = PyZenity.GetText()
    return float(response)


def resistor_max_actuation_readings(control_board, frequencies,
                                    oscope_reading_func):
    '''
    For each resistor in the high-voltage feed-back resistor bank, read the
    board measured voltage and the oscilloscope measured voltage for an
    actuation voltage that nearly saturates the feed-back resistor.

    By searching for an actuation voltage near saturation, the signal-to-noise
    ratio is minimized.
    '''
    # Set board amplifier gain to 1.
    # __NB__ This is likely _far_ lower than the actual gain _(which may be a
    # factor of several hundred)_..
    control_board.set_waveform_voltage(0)
    control_board.auto_adjust_amplifier_gain = False
    control_board.amplifier_gain = 1.

    # Set waveform voltage to a low value and obtain the corresponding
    # oscilloscope reading to calculate an approximate gain of the amplifier.
    target_voltage = 0.1
    control_board.set_waveform_voltage(target_voltage)
    oscope_rms = oscope_reading_func()
    estimated_amplifier_gain = oscope_rms / target_voltage

    # Based on the maximum amplified RMS voltage, define a set of actuation
    # voltages to search when performing calibration.
    max_post_gain_V = 150.  # TODO: Read this value from device.
    max_actuation_V = max_post_gain_V / estimated_amplifier_gain
    actuation_steps = np.linspace(0.005, max_actuation_V, num=100)

    # Define frequency/resistor index pairs to take measurements at.
    conditions = pd.DataFrame([[r, f] for r in range(2, -1, -1)
                               for f in frequencies],
                              columns=['resistor index', 'frequency'])

    # Define function to process each frequency/resistor index pair.
    def max_actuation_reading(x):
        '''
        Measure maximum board RMS voltage using specified feedback resistor, at
        the specified frequency.

        Request corresponding oscilloscope RMS voltage reading.
        '''
        r = x['resistor index'].values[0]
        f = x['frequency'].values[0]
        control_board.set_waveform_frequency(f)

        actuation_index, data = find_good(control_board, actuation_steps, r, 0,
                                          len(actuation_steps) - 1)
        board_measured_rms = data.loc[data['divider resistor index']
                                    >= 0, 'board measured V'].mean()
        oscope_rms = oscope_reading_func()
        return pd.DataFrame([[r, f, actuation_index, board_measured_rms,
                            oscope_rms]],
                            columns=['resistor index', 'frequency',
                                    'actuation index', 'board measured V',
                                    'oscope measured V'])

    # Return board-measured RMS voltage and oscilloscope-measured RMS voltage
    # for each frequency/feedback resistor pair.
    return (conditions.groupby(['resistor index', 'frequency'])
            .apply(max_actuation_reading).reset_index(drop=True))


if __name__ == '__main__':
    import numpy as np
    from dmf_control_board import *

    control_board = DMFControlBoard()
    control_board.connect()

    frequencies = np.logspace(2, np.log10(20e3), 10)
