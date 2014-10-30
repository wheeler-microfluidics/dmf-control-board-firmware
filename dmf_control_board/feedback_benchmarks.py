# coding: utf-8

from collections import OrderedDict
import time
import cPickle as pickle

import pandas as pd
from path import path
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import numpy as np
import scipy.optimize
import scipy.stats

from dmf_control_board import *

pd.set_option('display.width', 300)


frequencies = np.logspace(2, 13./3, 15) # frequencies to test
test_loads = [(0, 1e-12),
              (1, 1.5e-12),
              (2, 2.2e-12),
              (3, 3.3e-12),
              (4, 4.7e-12),
              (5, 6.8e-12),
              (6, 1e-11),
              (7, 1.5e-11),
              (8, 2.2e-11),
              (9, 3.3e-11),
              (10, 4.7e-11),
              (11, 6.8e-11),
              (12, 1e-10),
              (13, 1.5e-10),
              (14, 2.2e-10),
              (15, 3.3e-10),
              (16, 4.7e-10),
              (17, 6.8e-10),
              (18, 1e-9),]

n_repeats = 1
n_sampling_windows = 10
voltage = 100. # actuation voltage


def from_dictionary(data):
    df = get_test_data(data['frequencies'], data['test_loads'], data['n_repeats'], data['n_sampling_windows'])
    df.loc[:, 'V_hv'] = data['V_hv'].ravel()
    df.loc[:, 'V_fb'] = data['V_fb'].ravel()
    df.loc[:, 'hv_resistor'] = data['hv_resistor'].ravel().astype(int)
    df.loc[:, 'fb_resistor'] = data['fb_resistor'].ravel().astype(int)
    df.loc[:, 'C'] = data['C'].ravel()
    return df


def get_test_data(frequencies, test_loads, n_repeats, n_sampling_windows):
    df = pd.DataFrame({'frequency': np.zeros([len(frequencies) *
                                              len(test_loads)
                                              * n_repeats
                                              * n_sampling_windows],
                                             dtype=float)})
    df['V_actuation'] = voltage
    df['test_capacitor'] = 0.
    df['test_channel'] = 0.
    df['rms'] = True
    df['antialiasing_filter'] = True
    df['repeat_index'] = 0
    df['sample_index'] = 0
    df['V_hv'] = 0.
    df['hv_resistor'] = 0
    df['V_fb'] = 0.
    df['fb_resistor'] = 0
    df['C'] = 0.

    I = len(frequencies)
    J = len(test_loads)
    K = n_repeats

    frequency = df.frequency.values.reshape(I, -1)
    test_capacitor = df.test_capacitor.values.reshape(I, J, -1)
    test_channel = df.test_channel.values.reshape(I, J, -1)
    C = df.C.values.reshape(I, J, K, -1)
    V_hv = df.V_hv.values.reshape(I, J, K, -1)
    V_fb = df.V_fb.values.reshape(I, J, K, -1)
    hv_resistor = df.hv_resistor.values.reshape(I, J, K, -1)
    fb_resistor = df.fb_resistor.values.reshape(I, J, K, -1)
    repeat_index = df.repeat_index.values.reshape(I, J, K, -1)
    sample_index = df.sample_index.values.reshape(I, J, K, -1)

    for i, f in enumerate(frequencies):
    #     proxy.set_waveform_frequency(f)
        frequency[i] = f
        for j, (channel, C_device) in enumerate(test_loads):
            test_capacitor[i, j] = C_device
            test_channel[i, j] = channel
            for k in range(n_repeats):
    #             print "%.2fkHz, C=%.2fpF, rep=%d" % (f/1e3, 1e12*C_device, k)
    #             state = np.zeros(proxy.number_of_channels())
    #             state[channel] = 1
    #             results = proxy.measure_impedance(10.0, n_sampling_windows, 0, True, rms, state)
                C[i, j, k, :] = np.random.rand(n_sampling_windows) # results.capacitance()
                V_hv[i, j, k, :] = np.random.rand(n_sampling_windows) # results.V_hv
                V_fb[i, j, k, :] = np.random.rand(n_sampling_windows) # results.V_fb
                hv_resistor[i, j, k, :] = np.random.randint(0, 2, size=n_sampling_windows) # results.hv_resistor
                fb_resistor[i, j, k, :] = np.random.randint(0, 2, size=n_sampling_windows) # results.fb_resistor
                repeat_index[i, j, k, :] = k
                sample_index[i, j, k, :] = range(n_sampling_windows)
    return df


def run_experiment(exp_name, test_loads=None, frequencies=None, use_antialiasing_filter=None, rms=None):
    # if cached data exists, load it
    if (cachedir / path(exp_name + '.pickle')).exists():
        with open(cachedir / path(exp_name + '.pickle'), 'rb') as f:
            data = pickle.load(f)
        data['exp_name'] = exp_name
        return data
    else:

        if use_antialiasing_filter is None:
            use_antialiasing_filter = True

        proxy = DMFControlBoard()
        proxy.connect()
        proxy.use_antialiasing_filter = use_antialiasing_filter
        proxy.connect()

        if test_loads is None:
            test_loads = [(0, 1e-12),
                          (1, 1.5e-12),
                          (2, 2.2e-12),
                          (3, 3.3e-12),
                          (4, 4.7e-12),
                          (5, 6.8e-12),
                          (6, 1e-11),
                          (7, 1.5e-11),
                          (8, 2.2e-11),
                          (9, 3.3e-11),
                          (10, 4.7e-11),
                          (11, 6.8e-11),
                          (12, 1e-10),
                          (13, 1.5e-10),
                          (14, 2.2e-10),
                          (15, 3.3e-10),
                          (16, 4.7e-10),
                          (17, 6.8e-10),
                          (18, 1e-9),
            ]

        if frequencies is None:
            frequencies = np.logspace(2, 13./3, 15) # frequencies to test

        if rms is None:
            rms = True

        voltage = 100 # actuation voltage
        n_repeats = 1 # number of repeated/independent measurements for each condition
        n_sampling_windows = 10 # number of sampling windows per measurement

        # prepare the output variables
        C = np.zeros([len(frequencies), len(test_loads), n_repeats, n_sampling_windows])
        V_hv = np.zeros([len(frequencies), len(test_loads), n_repeats, n_sampling_windows])
        V_fb = np.zeros([len(frequencies), len(test_loads), n_repeats, n_sampling_windows])
        hv_resistor = np.zeros([len(frequencies), len(test_loads), n_repeats, n_sampling_windows])
        fb_resistor = np.zeros([len(frequencies), len(test_loads), n_repeats, n_sampling_windows])

        proxy.set_waveform_voltage(voltage)

        for i, f in enumerate(frequencies):
            proxy.set_waveform_frequency(f)

            for j, (channel, C_device) in enumerate(test_loads):
                for k in range(n_repeats):
                    print "%.2fkHz, C=%.2fpF, rep=%d" % (f/1e3, 1e12*C_device, k)
                    state = np.zeros(proxy.number_of_channels())
                    state[channel] = 1
                    results = proxy.measure_impedance(10.0, n_sampling_windows, 0, True, rms, state)
                    C[i,j,k,:] = results.capacitance()
                    V_hv[i,j,k,:] = results.V_hv
                    V_fb[i,j,k,:] = results.V_fb
                    hv_resistor[i,j,k,:] = results.hv_resistor
                    fb_resistor[i,j,k,:] = results.fb_resistor

        calibration = proxy.calibration

        # set all channels back to zero
        proxy.set_state_of_all_channels(np.zeros(proxy.number_of_channels()))

        # store all experimental variables (inputs and outputs) in a dictionary
        data = dict(test_loads=test_loads, frequencies=frequencies, voltage=voltage, n_repeats=n_repeats,
                    n_sampling_windows=n_sampling_windows, C=C, V_hv=V_hv, V_fb=V_fb,
                    fb_resistor=fb_resistor, hv_resistor=hv_resistor, software_version=proxy.software_version(),
                    calibration=calibration)

        # if the cachedir doesn't exist, create it
        if not cachedir.exists():
            cachedir.makedirs_p()

        # pickle the data
        with open(cachedir / path(exp_name + '.pickle'), 'wb') as f:
            pickle.dump(data, f, -1)

        data['exp_name'] = exp_name
        return data


def fit_fb_calibration2(data):
    '''
    Fit feedback calibration data to solve for values of `C_fb[:]` and `R_fb[:]`.
    '''
    calibration = data['calibration']

    df = from_dictionary(data)

    # set initial guesses for the feedback parameters
    R_fb = pd.Series([2e2, 2e3, 2e4, 2e5, 2e6])
    C_fb = pd.Series(len(calibration.C_fb) * [50e-12])

    # error function
    def error(p0, df, calibration):
        # Impedance of the reference resistor on the HV attenuator circuit.
        Z = 10e6
        R_fb = p0[0]

        # If the parameter vector only contains one variable, the capacitance
        # is zero
        if len(p0) == 2:
            C_fb = p0[1]
        else:
            C_fb = 0

        R_hv = calibration.R_hv[df.hv_resistor.values]
        C_hv = calibration.C_hv[df.hv_resistor.values]

        if calibration.hw_version.major == 1:
            V_actuation = df.V_hv / np.abs(1 / (Z / R_hv + 1 + Z * 2 * np.pi
                                                * C_hv * complex(0, 1) *
                                                df.frequency))
            return (df.V_fb - df.V_actuation * R_fb * df.C * 2 * np.pi * f /
                    np.sqrt(1 + np.square(2 * np.pi * R_fb *
                                          (C_fb + df.C) * df.frequency)))
        else:
            V_actuation = df.V_hv / np.abs(1 / (Z / R_hv + Z * 2 * np.pi * C_hv *
                                             complex(0, 1) * df.frequency))
            return (df.V_fb - df.V_actuation * R_fb * df.C * 2 * np.pi *
                    df.frequency / np.sqrt(1 + np.square(2 * np.pi * R_fb *
                                                         C_fb *
                                                         df.frequency)))

    # Perform a nonlinear least-squares fit of the data.
    def fit_model(p0, df, calibration):
        p1, cov_x, infodict, mesg, ier = scipy.optimize.leastsq(
            error, p0, args=(df, calibration), full_output=True)
        p1 = np.abs(p1)
        E = error(p1, df, calibration)
        return p1, E, cov_x

    CI = []

    # Fit feedback parameters for each feedback resistor.
    for i in range(len(calibration.R_fb)):
        # Only include data points for the given feedback resistor (and where
        # `hv_resistor` is a valid index).
        df_i = df.loc[(df.fb_resistor == i)].dropna()

        if df_i.shape[0] < 2:
            continue

        print "feedback_resistor=%d: n_data_points=%d" % (i, df_i.shape[0])

        # Fit the data assuming no parasitic capacitance (model 1).
        p0_1 = [R_fb[i]]
        p1_1, E_1, cov_x_1 = fit_model(p0_1, df_i, calibration)
        df_1 = (len(E_1) - len(p0_1))
        chi2_1 = np.sum(E_1 ** 2)
        chi2r_1 = chi2_1 / (df_1 - 1)

        # fit the data including parasitic capacitance (model 2)
        p0_2 = [R_fb[i], C_fb[i]]
        p1_2, E_2, cov_x_2 = fit_model(p0_2, df_i, calibration)
        df_2 = (len(E_2) - len(p0_2))
        chi2_2 = np.sum(E_2 ** 2)
        chi2r_2 = chi2_2 / (df_2 - 1)

        # do an F-test to compare the models
        F = (chi2_1 - chi2_2) / chi2r_2
        p_value = scipy.stats.f.cdf(F, 1, df_2-1)
        print "F = %.3e, p_value=%.3f" % (F, p_value)

        # if the p_value is > 0.95, we assume that the capacitive term is
        # necessary
        if p_value > .95 and cov_x_2 is not None:
            calibration.R_fb[i] = p1_2[0]
            calibration.C_fb[i] = p1_2[1]
            CI.append((100 * np.sqrt(chi2r_2 * np.diag(cov_x_2)) / p1_2))
            print "R_fb[%d] = %.3e +/- %.3f %%" % (i, p1_2[0], CI[i][0])
            print "C_fb[%d] = %.3e +/- %.3f %%" % (i, p1_2[1], CI[i][1])
            print "sqrt(chi2r*sigma^2) = %.1f mV" % (1e3 * np.sqrt(chi2r_2))
        else: # otherwise, set the capacitance to zero
            calibration.R_fb[i] = p1_1[0]
            calibration.C_fb[i] = 0
            CI.append((100 * np.sqrt(chi2r_1 * np.diag(cov_x_1)) /
                       p1_1).tolist() + [0])
            print "R_fb[%d] = %.3e +/- %.3f %%" % (i, p1_1[0], CI[i][0])
            print "sqrt(chi2r*sigma^2) = %.1f mV" % (1e3 * np.sqrt(chi2r_1))

        print

    # print the feedback parameter estimates
    print "Feedback calibration values:"
    for i in range(len(calibration.R_fb)):
        print "R_fb[%d] = %.3e Ohms +/- %.3f %%" % (i, calibration.R_fb[i], CI[i][0])
    for i in range(len(calibration.R_fb)):
        print "C_fb[%d] = %.3e F +/- %.3f %%" % (i, calibration.C_fb[i], CI[i][1])

    cleaned_df = df.dropna()
    grouped = cleaned_df.groupby(['frequency', 'test_channel', 'repeat_index'])

    for (f, channel, repeat_index), group in grouped:
        r = FeedbackResults(group.V_actuation.iloc[0], f, 5.0,
                            group.V_hv.values, group.hv_resistor.values,
                            group.V_fb.values, group.fb_resistor.values,
                            calibration)
        df.loc[group.index, 'C'] = r.capacitance()
    return df


def fit_fb_calibration(data):
    '''
    Fit feedback calibration data to solve for values of `C_fb[:]` and `R_fb[:]`.
    '''
    test_loads = data['test_loads']
    frequencies = data['frequencies']
    voltage = data['voltage']
    n_repeats = data['n_repeats']
    C = data['C']
    V_hv = data['V_hv']
    V_fb = data['V_fb']
    hv_resistor = data['hv_resistor']
    fb_resistor = data['fb_resistor']
    calibration = data['calibration']

    # create C_device matrix to match shape of V_fb
    C_device = np.array([x for channel, x in test_loads])
    C_device = np.tile(np.reshape(C_device, [1, len(C_device)] + [1]*(len(V_fb.shape) - 2)),
                       [V_fb.shape[0]] + [1] + list(V_fb.shape[2:]))

    # Create frequency matrix to match shape of `V_fb`.
    f = np.tile(np.reshape(frequencies, [len(frequencies)] + [1]*(len(V_fb.shape) - 1)),
                [1] + list(V_fb.shape[1:]))

    # set initial guesses for the feedback parameters
    R_fb = [2e2, 2e3, 2e4, 2e5, 2e6]
    C_fb = len(calibration.C_fb)*[50e-12]

    # error function
    def error(p0, V_hv, hv_resistor, V_fb, f, C, calibration):
        # Impedance of the reference resistor on the HV attenuator circuit.
        Z = 10e6
        R_fb = p0[0]

        # If the parameter vector only contains one variable, the capacitance
        # is zero
        if len(p0) == 2:
            C_fb = p0[1]
        else:
            C_fb = 0

        R_hv = calibration.R_hv[hv_resistor.astype(int)]
        C_hv = calibration.C_hv[hv_resistor.astype(int)]

        if calibration.hw_version.major == 1:
            V_actuation = V_hv / np.abs(1 / (Z / R_hv + 1 + Z * 2 * np.pi *
                                             C_hv * complex(0, 1) * f))
            return (V_fb - V_actuation * R_fb * C * 2 * np.pi * f /
                    np.sqrt(1 + np.square(2 * np.pi * R_fb * (C_fb + C) * f)))
        else:
            V_actuation = V_hv / np.abs(1 / (Z / R_hv + Z * 2 * np.pi * C_hv *
                                             complex(0, 1) * f))
            return (V_fb - V_actuation * R_fb * C * 2 * np.pi * f /
                    np.sqrt(1 + np.square(2 * np.pi * R_fb * C_fb * f)))

    # Perform a nonlinear least-squares fit of the data.
    def fit_model(p0, V_hv, hv_resistor, V_fb, f, C_device, calibration):
        p1, cov_x, infodict, mesg, ier = scipy.optimize.leastsq(
            error, p0, args=(V_hv, hv_resistor, V_fb, f, C_device,
                             calibration), full_output=True)
        p1 = np.abs(p1)
        E = error(p1, V_hv, hv_resistor, V_fb, f, C_device, calibration)
        return p1, E, cov_x

    CI = []

    # Fit feedback parameters for each feedback resistor.
    for i in range(len(calibration.R_fb)):
        # Only include data points for the given feedback resistor (and where
        # `hv_resistor` is a valid index).
        ind = mlab.find(np.logical_and(fb_resistor == i, hv_resistor >= 0))

        if len(ind) < 2:
            continue

        print "feedback_resistor=%d: n_data_points=%d" % (i, len(ind))

        # Fit the data assuming no parasitic capacitance (model 1).
        p0_1 = [R_fb[i]]
        p1_1, E_1, cov_x_1 = fit_model(p0_1, V_hv.flatten()[ind],
                                       hv_resistor.flatten()[ind],
                                       V_fb.flatten()[ind], f.flatten()[ind],
                                       C_device.flatten()[ind], calibration)
        df_1 = (len(E_1)-len(p0_1))
        chi2_1 = np.sum(E_1**2)
        chi2r_1= chi2_1/(df_1-1)

        # fit the data including parasitic capacitance (model 2)
        p0_2 = [R_fb[i], C_fb[i]]
        p1_2, E_2, cov_x_2 = fit_model(p0_2, V_hv.flatten()[ind],
                                       hv_resistor.flatten()[ind],
                                       V_fb.flatten()[ind], f.flatten()[ind],
                                       C_device.flatten()[ind], calibration)
        df_2 = (len(E_2) - len(p0_2))
        chi2_2 = np.sum(E_2 ** 2)
        chi2r_2 = chi2_2 / (df_2 - 1)

        # do an F-test to compare the models
        F = (chi2_1 - chi2_2) / chi2r_2
        p_value = scipy.stats.f.cdf(F, 1, df_2-1)
        print "F = %.3e, p_value=%.3f" % (F, p_value)

        # if the p_value is > 0.95, we assume that the capacitive term is
        # necessary
        if p_value > .95 and cov_x_2 is not None:
            calibration.R_fb[i] = p1_2[0]
            calibration.C_fb[i] = p1_2[1]
            CI.append((100 * np.sqrt(chi2r_2 * np.diag(cov_x_2)) / p1_2))
            print "R_fb[%d] = %.3e +/- %.3f %%" % (i, p1_2[0], CI[i][0])
            print "C_fb[%d] = %.3e +/- %.3f %%" % (i, p1_2[1], CI[i][1])
            print "sqrt(chi2r*sigma^2) = %.1f mV" % (1e3 * np.sqrt(chi2r_2))
        else: # otherwise, set the capacitance to zero
            calibration.R_fb[i] = p1_1[0]
            calibration.C_fb[i] = 0
            CI.append((100 * np.sqrt(chi2r_1 * np.diag(cov_x_1)) /
                       p1_1).tolist() + [0])
            print "R_fb[%d] = %.3e +/- %.3f %%" % (i, p1_1[0], CI[i][0])
            print "sqrt(chi2r*sigma^2) = %.1f mV" % (1e3 * np.sqrt(chi2r_1))

        print

    # print the feedback parameter estimates
    print "Feedback calibration values:"
    for i in range(len(calibration.R_fb)):
        print "R_fb[%d] = %.3e Ohms +/- %.3f %%" % (i, calibration.R_fb[i], CI[i][0])
    for i in range(len(calibration.R_fb)):
        print "C_fb[%d] = %.3e F +/- %.3f %%" % (i, calibration.C_fb[i], CI[i][1])

    print

    # update capacitance values based on new fit
    for i, frequency in enumerate(frequencies):
        for j, (channel, C_device) in enumerate(test_loads):
            for k in range(n_repeats):
                results = FeedbackResults(voltage, frequency, 5.0, V_hv[i,j,k,:],
                                          hv_resistor[i,j,k,:].astype(int), V_fb[i,j,k,:],
                                          fb_resistor[i,j,k,:].astype(int), calibration)
                C[i,j,k,:] = results.capacitance()
    data['C'] = C
    data['calibration'] = calibration
    return data


# In[4]:

def update_fb_calibration(calibration):
    print "Updating feedback calibration values...\n"
    proxy = DMFControlBoard()
    proxy.connect()

    # write new calibration parameters to the control board
    for i in range(0, len(calibration.R_fb)):
        proxy.set_series_resistor_index(1, i)
        proxy.set_series_resistance(1, calibration.R_fb[i])
        proxy.set_series_capacitance(1, calibration.C_fb[i])
    # reconnect to update settings
    proxy.connect()


# In[5]:

def plot_capacitance_vs_frequency(data):
    test_loads = data['test_loads']
    frequencies = data['frequencies']
    C = data['C']
    fb_resistor = data['fb_resistor']
    calibration = data['calibration']

    # create a masked array version of the capacitance matrix
    C = np.ma.masked_invalid(C)

    # create frequency matrix to match shape of C
    f = np.tile(np.reshape(frequencies, [len(frequencies)] + [1]*(len(C.shape) - 1)),
                [1] + list(C.shape[1:]))

    # plot the capacitance of each experiment vs frequency (with the data points
    # color-coded according to the feedback resistor)
    plt.figure(figsize=figsize)
    legend = []
    for i in range(len(calibration.R_fb)):
        legend.append("R$_{fb,%d}$" % i)
        ind = mlab.find(fb_resistor == i)
        plt.loglog(f.flatten()[ind], C.flatten()[ind], 'o')
    plt.xlim(0.8*np.min(frequencies), 1.2*np.max(frequencies))
    plt.ylim(0.8*np.min(C), 1.2*np.max(C))
    for C_device in [C_device for channel, C_device in test_loads]:
        plt.plot(frequencies, C_device*np.ones(len(frequencies)), '--', color='0.5')
    plt.legend(legend)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('C$_{device}$ (F)')
    plt.title('C$_{device}$')
    plt.tight_layout()


# In[6]:

def estimate_relative_error_in_nominal_capacitance(data):
    test_loads = data['test_loads']
    C = data['C']
    C = np.ma.masked_invalid(C)

    # Calculate the relative difference in the mean capacitance values measured
    # relative to the nominal values.
    C_device = np.array([x for channel, x in test_loads])
    C_est = np.mean(np.mean(np.mean(C, 3), 2), 0)
    print ('Estimated relative error in nominal capacitance values = %.1f%% '
           ' +/-%.1f%%' % (np.mean(100 * (C_est - C_device) / C_device),
                           np.std(100 * (C_est - C_device) / C_device)))
    print 100 * (C_est - C_device) / C_device
    print


def plot_impedance_vs_frequency(data):
    test_loads = data['test_loads']
    frequencies = data['frequencies']
    C = data['C']
    fb_resistor = data['fb_resistor']
    calibration = data['calibration']

    # create a masked array version of the capacitance matrix
    C = np.ma.masked_invalid(C)

    # create frequency matrix to match shape of C
    f = np.tile(np.reshape(frequencies, [len(frequencies)] + [1]*(len(C.shape) - 1)),
                [1] + list(C.shape[1:]))

    # plot the impedance of each experiment vs frequency (with the data points
    # color-coded according to the feedback resistor)
    plt.figure(figsize=figsize)
    legend = []
    for i in range(len(calibration.R_fb)):
        legend.append("R$_{fb,%d}$" % i)
        ind = mlab.find(fb_resistor == i)
        plt.loglog(f.flatten()[ind], 1.0 / (2 * np.pi * C.flatten()[ind] * f.flatten()[ind]), 'o')
    plt.xlim(0.8*np.min(frequencies), 1.2*np.max(frequencies))
    for C_device in [C_device for channel, C_device in test_loads]:
        plt.plot(frequencies, 1.0 / (2 * np.pi * C_device*np.ones(len(frequencies)) * frequencies),
                 '--', color='0.5')
    plt.legend(legend)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Z$_{device}$ ($\Omega$)')
    plt.title('Z$_{device}$')
    plt.tight_layout()


# In[8]:

def calculate_stats(data):
    test_loads = data['test_loads']
    frequencies = data['frequencies']
    C = data['C']

    # create a masked array version of the capacitance matrix
    C = np.ma.masked_invalid(C)

    # create C_nominal matrix to match shape of C
    C_nominal = np.array([x for channel, x in test_loads])
    C_nominal = np.tile(np.reshape(C_nominal, [1, len(C_nominal)] + [1]*(len(C.shape) - 2)),
                       [C.shape[0]] + [1] + list(C.shape[2:]))

    # figure out the mean, median, std, CV, RMSE and bias of the measured capacitance values
    mean_C = np.zeros((len(frequencies), len(test_loads)))
    median_C = np.zeros((len(frequencies), len(test_loads)))
    std_C = np.zeros((len(frequencies), len(test_loads)))
    RMSE_C = np.zeros((len(frequencies), len(test_loads)))
    bias_C = np.zeros((len(frequencies), len(test_loads)))
    for i in range(len(frequencies)):
        for j in range(len(test_loads)):
            mean_C[i,j] = np.ma.masked_invalid(C[i,j,:]).mean()
            median_C[i,j] = np.median(np.ma.masked_invalid(C[i,j,:]))
            std_C[i,j] = np.ma.masked_invalid(C[i,j,:]).std()
            RMSE_C[i,j] = 100*np.sqrt(np.ma.masked_invalid((C[i,j,:] - C_nominal[i,j,:])**2).mean())/mean_C[i,j]
            bias_C[i,j] = 100*np.ma.masked_invalid(C[i,j,:] - C_nominal[i,j,:]).mean()/mean_C[i,j]
    cv_C = 100*np.ma.masked_invalid(std_C/mean_C)
    RMSE_C = np.ma.masked_invalid(RMSE_C)
    bias_C = np.ma.masked_invalid(bias_C)
    mean_C = np.ma.masked_invalid(mean_C)
    median_C = np.ma.masked_invalid(median_C)
    std_C = np.ma.masked_invalid(std_C)

    stats = {'mean': mean_C, 'median': median_C, 'std': std_C, 'CV': cv_C, 'bias': bias_C, 'RMSE': RMSE_C}
    for k, v in stats.items():
        if k in ('CV', 'bias', 'RMSE'):
            print (k + "(C) = %.1f%% +/- %.1f%% (median=%.1f%%, max=%.1f%%)" % (np.mean(v), np.std(v), np.median(v), np.max(v)))
    print

    return stats


# In[9]:

def plot_histogram(title, param, vmin=None, vmax=None):
    # plot histograms of the CV, RMSE and bias of C
    if vmax is None:
        vmax = np.max(param)
    if vmin is None:
        vmin = np.min(param)
    plt.hist(param.flatten(), np.linspace(vmin, vmax, 50))
    plt.xlim(vmin, vmax)
    plt.xlabel(title)


# In[10]:

def print_detailed_stats_by_condition(data, stats):
    test_loads = data['test_loads']
    frequencies = data['frequencies']
    mean = stats['mean']
    CV = stats['CV']
    bias = stats['bias']
    RMSE = stats['RMSE']

    # print the RMSE, CV, and bias for each test capacitor and frequency combination
    for i, (channel, C_device) in enumerate(test_loads):
        print "\n%.2f pF" % (C_device*1e12)
        for j in range(len(frequencies)):
            print "%.1fkHz: mean(C)=%.2f pF, RMSE=%.1f%%, CV=%.1f%%, bias=%.1f%%" % (frequencies[j]/1e3,
                                                       1e12*mean[j,i],
                                                       RMSE[j,i],
                                                       CV[j,i],
                                                       bias[j,i])
    print


# In[11]:

def plot_measured_vs_nominal_capacitance_for_each_frequency(data, stats):
    # plot the measured vs nominal capacitance for each frequency
    frequencies = data['frequencies']
    test_loads = data['test_loads']
    mean_C = stats['mean']
    std_C = stats['std']

    for i in range(len(frequencies)):
        plt.figure()
        plt.title('(frequency=%.2fkHz)' % (frequencies[i]/1e3))
        for j, (channel, C_device) in enumerate(test_loads):
            plt.errorbar(C_device, mean_C[i,j],
                         std_C[i,j], fmt='k')
        C_device = np.array([x for channel, x in test_loads])
        plt.loglog(C_device, C_device, 'k:')
        plt.xlim(min(C_device)*.9, max(C_device)*1.1)
        plt.ylim(min(C_device)*.9, max(C_device)*1.1)
        plt.xlabel('C$_{nom}$ (F)')
        plt.ylabel('C$_{measured}$ (F)')


# In[12]:

def plot_colormap(title, C_nominal, frequencies, param, vmin=None, vmax=None):
    # plot colormaps of the RMSE, CV and bias (test capacitance vs frequency)
    plt.pcolormesh(np.transpose(param), vmin=vmin, vmax=vmax)
    plt.set_cmap('hot')
    plt.colorbar()
    plt.title(title)
    plt.xlabel('Frequency')
    plt.ylabel('Capacitance')
    plt.xticks(np.arange(len(frequencies))[::3] + 0.5, ["%.2fkHz" % (f/1e3) for f in frequencies][::3])
    plt.yticks(np.arange(len(C_nominal))[::2] + 0.5, ["%.2fpF" % (c*1e12) for c in C_nominal][::2])
    plt.setp(plt.xticks()[1], rotation=90)
    plt.xlim(0, len(frequencies))
    plt.ylim(0, len(C_nominal))


if __name__ == '__main__':
    cachedir = path('.cache/2014.10.22/002')

    data = run_experiment('7 caps, 100V, 100Hz-22kHz, rms, with filter (switching board v2.1)')
    data = fit_fb_calibration(data)
    df = fit_fb_calibration2(data)

    # Write new calibrated feedback resistor and capacitor values to control
    # board configuration.
    #update_fb_calibration(data['calibration'])
    #import pudb; pudb.set_trace()
    estimate_relative_error_in_nominal_capacitance(data)
    #plot_capacitance_vs_frequency(data)
    #plot_impedance_vs_frequency(data)

    #stats = calculate_stats(data)
    ##print_detailed_stats_by_condition(data, stats)
    ##plot_measured_vs_nominal_capacitance_for_each_frequency(data, stats)

    #plot_histogram('RMSE(C) (%)', stats['RMSE'], vmax=vmax)
    #plot_histogram('bias(C) (%)', stats['bias'], vmin=vmin, vmax=vmax)

    #C_nominal = np.array([x for channel, x in data['test_loads']])
    #plot_colormap('RMSE(C) (%)', C_nominal, data['frequencies'], stats['RMSE'], vmax=vmax )
    #plot_colormap('CV(C) (%)', C_nominal, data['frequencies'], stats['CV'], vmax=vmax)
    #plot_colormap('abs(bias(C)) (%)', C_nominal, data['frequencies'], np.abs(stats['bias']), vmax=vmax)
