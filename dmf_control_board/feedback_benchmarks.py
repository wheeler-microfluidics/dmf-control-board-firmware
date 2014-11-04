# coding: utf-8

from collections import OrderedDict
import time
import cPickle as pickle

import pandas as pd
from path import path
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
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
    df = get_test_frame(data['frequencies'], data['test_loads'],
                        data['n_repeats'], data['n_sampling_windows'])
    df.loc[:, 'V_hv'] = data['V_hv'].ravel()
    df.loc[:, 'V_fb'] = data['V_fb'].ravel()
    df.loc[:, 'hv_resistor'] = data['hv_resistor'].ravel().astype(int)
    df.loc[:, 'fb_resistor'] = data['fb_resistor'].ravel().astype(int)
    df.loc[:, 'C'] = data['C'].ravel()
    return df


def get_test_frame(frequencies, test_loads, n_repeats, n_sampling_windows):
    '''
    Return a `pandas.DataFrame`, ready to store one measurement per row.
    Upon returning, the following columns are populated:

     - `frequency`: The frequency of the waveform to use during measurement.
     - `test_capacitor`: The test capacitor to use during measurement.
     - `test_channel`: The test capacitor channel to use during measurement.
     - `repeat_index`: The repetition count for the corresponding
       frequency/capacitor measurement set.
     - `sample_index`: The index of the sample in each repetition set.

    The remaining columns are initialized to starting values, but should be
    overwritten by the `run_experiment` function.
    '''
    sample_index = np.tile(range(n_sampling_windows), n_repeats *
                           len(test_loads) * len(frequencies))
    repeat_index = np.tile(np.repeat(range(n_repeats), n_sampling_windows),
                           len(test_loads) * len(frequencies))
    channel = np.tile(np.repeat(zip(*test_loads)[0], n_repeats *
                                n_sampling_windows), len(frequencies))
    capacitor = np.tile(np.repeat(zip(*test_loads)[1], n_repeats *
                                  n_sampling_windows), len(frequencies))
    frequency = np.repeat(frequencies, len(test_loads) * n_repeats *
                          n_sampling_windows)

    df = pd.DataFrame({'frequency': frequency})
    df['test_capacitor'] = capacitor
    df['test_channel'] = channel
    df['repeat_index'] = repeat_index
    df['sample_index'] = sample_index
    df['V_actuation'] = voltage
    df['rms'] = True
    df['antialiasing_filter'] = True
    df['V_hv'] = 0.
    df['hv_resistor'] = 0
    df['V_fb'] = 0.
    df['fb_resistor'] = 0
    df['C'] = 0.

    return df


def run_experiment(proxy, test_loads=None, frequencies=None,
                   use_antialiasing_filter=None, rms=None):
    if use_antialiasing_filter is None:
        use_antialiasing_filter = True
    # Update device anti-aliasing setting and reset the device to take effect.
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
                      (18, 1e-9),]

    if frequencies is None:
        frequencies = np.logspace(2, 13. / 3, 15)  # frequencies to test

    if rms is None:
        rms = True

    # Actuation voltage.
    voltage = 100
    # Number of repeated/independent measurements for each condition.
    n_repeats = 1
    # Number of sampling windows per measurement.
    n_sampling_windows = 10

    # Prepare the test output `pandas.DataFrame`.
    test_frame = get_test_frame(frequencies, test_loads, n_repeats,
                                n_sampling_windows)

    proxy.set_waveform_voltage(voltage)

    previous_frequency = None
    for (f, C_device, channel, i), group in grouped:
        if frequency != previous_frequency:
            proxy.set_waveform_frequency(frequency)
        print "%.2fkHz, C=%.2fpF, rep=%d" % (f / 1e3, 1e12 * C_device, i)
        state = np.zeros(proxy.number_of_channels())
        state[channel] = 1
        results = proxy.measure_impedance(10.0, n_sampling_windows, 0,
                                            True, rms, state)
        group.loc[:, 'C'] = results.capacitance()
        group.loc[:, 'V_hv'] = results.V_hv
        group.loc[:, 'V_fb'] = results.V_fb
        group.loc[:, 'hv_resistor'] = results.hv_resistor
        group.loc[:, 'hv_resistor'] = results.fb_resistor

    calibration = proxy.calibration

    # Set all channels back to zero
    proxy.set_state_of_all_channels(np.zeros(proxy.number_of_channels()))

    return test_frame, calibration


def fit_fb_calibration(df, calibration):
    '''
    Fit feedback calibration data to solve for values of `C_fb[:]` and
    `R_fb[:]`.

    Returns a `pandas.DataFrame` indexed by the feedback resistor/capacitance
    index, and with the following columns:
     - Model: Either with parasitic capacitance term or not.
     - N: Number of samples used for fit.
     - F: F-value
     - p-value: p-value from Chi squared test.
     - R_fb: Feedback resistor value based on fit.
     - R-CI %: Confidence interval for feedback resistor value.
     - C_fb: Feedback capacitor value based on fit (0 if no-capacitance model
       is used).
     - C-CI %: Confidence interval for feedback capacitance value.

    __N.B.__ This function does not actually _update_ the calibration, it only
    performs the fit.
    See `apply_calibration`.
    '''
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
            V_actuation = df.V_hv / np.abs(1 / (Z / R_hv + Z * 2 * np.pi * C_hv
                                                * complex(0, 1) *
                                                df.frequency))
            return (df.V_fb - df.V_actuation * R_fb * df.C * 2 * np.pi *
                    df.frequency / np.sqrt(1 + np.square(2 * np.pi * R_fb *
                                                         C_fb * df.frequency)))

    # Perform a nonlinear least-squares fit of the data.
    def fit_model(p0, df, calibration):
        p1, cov_x, infodict, mesg, ier = scipy.optimize.leastsq(
            error, p0, args=(df, calibration), full_output=True)
        p1 = np.abs(p1)
        E = error(p1, df, calibration)
        return p1, E, cov_x

    CI = []

    feedback_records = []
    # Fit feedback parameters for each feedback resistor.
    for i in range(len(calibration.R_fb)):
        # Only include data points for the given feedback resistor (and where
        # `hv_resistor` is a valid index).
        df_i = df.loc[(df.fb_resistor == i)].dropna()

        if df_i.shape[0] < 2:
            continue

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

        # if the p_value is > 0.95, we assume that the capacitive term is
        # necessary
        if p_value > .95 and cov_x_2 is not None:
            model = 'w/Parasitic C'
            chi2r = chi2r_2
            R_fb_i = p1_2[0]
            C_fb_i = p1_2[1]
            CI.append((100 * np.sqrt(chi2r_2 * np.diag(cov_x_2)) / p1_2))
        else:  # otherwise, set the capacitance to zero
            model = 'w/o Parasitic C'
            chi2r = chi2r_2
            R_fb_i = p1_1[0]
            C_fb_i = 0
            CI.append((100 * np.sqrt(chi2r_1 * np.diag(cov_x_1)) /
                       p1_1).tolist() + [0])
        feedback_records.append([model, df_i.shape[0], R_fb_i, CI[i][0],
                                 C_fb_i, CI[i][1], F, (1e3 * np.sqrt(chi2r)),
                                 p_value])

    calibration_df = pd.DataFrame(feedback_records,
                                  columns=['Model', 'N', 'R_fb', 'R-CI %',
                                           'C_fb', 'C-CI %', 'F',
                                           'sqrt(Chi2r*sigma^2)', 'p-value'])
    return calibration_df


def apply_calibration(df, calibration_df, calibration):
    '''
    Apply calibration values from `fit_fb_calibration` result to `calibration`
    object.
    '''
    for i, (R_fb, C_fb) in calibration_df[['R_fb', 'C_fb']].iterrows():
        calibration.R_fb[i] = R_fb
        calibration.C_fb[i] = C_fb

    cleaned_df = df.dropna()
    grouped = cleaned_df.groupby(['frequency', 'test_channel', 'repeat_index'])

    for (f, channel, repeat_index), group in grouped:
        r = FeedbackResults(group.V_actuation.iloc[0], f, 5.0,
                            group.V_hv.values, group.hv_resistor.values,
                            group.V_fb.values, group.fb_resistor.values,
                            calibration)
        # Update the measured capacitance values based on the updated
        # calibration model.
        df.loc[group.index, 'C'] = r.capacitance()


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


def plot_capacitance_vs_frequency(df, **kwargs):
    cleaned_df = df.dropna().copy()
    fb_resistor_df = cleaned_df.set_index(cleaned_df.fb_resistor)

    axis = kwargs.pop('axis', None)
    s = kwargs.pop('s', 50)
    facecolor = kwargs.pop('facecolor', 'none')
    if axis is None:
        fig = plt.figure()
        axis = fig.add_subplot(111)
    stats = fb_resistor_df[['frequency', 'C']].describe()
    axis.set_xlim(0.8 * stats.frequency['min'], 1.2 * stats.frequency['max'])
    axis.set_ylim(0.8 * stats.C['min'], 1.2 * stats.C['max'])

    frequencies = fb_resistor_df.frequency.unique()
    # Plot nominal test capacitance lines.
    for C in fb_resistor_df.test_capacitor.unique():
        axis.plot(frequencies, [C] * len(frequencies), '--', alpha=0.7,
                  color='0.5', linewidth=1)
    colors = axis._get_lines.color_cycle
    # Plot scatter of _measured_ capacitance vs. frequency.
    for k, v in fb_resistor_df[['frequency', 'C']].groupby(level=0):
        v.plot(kind='scatter', x='frequency', y='C', loglog=True,
               label='R$_{fb,%d}$' % k, ax=axis, color=colors.next(),
               s=s, facecolor=facecolor, **kwargs)
    axis.legend(loc='upper right')
    axis.set_xlabel('Frequency (Hz)')
    axis.set_ylabel('C$_{device}$ (F)')
    axis.set_title('C$_{device}$')
    plt.tight_layout()
    return axis


def estimate_relative_error_in_nominal_capacitance(df):
    # Calculate the relative percentage difference in the mean capacitance
    # values measured relative to the nominal values.
    cleaned_df = df.dropna().copy()
    C_relative_error = (cleaned_df.groupby('test_capacitor')
                        .apply(lambda x: ((x['C'] - x['test_capacitor']) /
                               x['test_capacitor']).describe()))
    pd.set_eng_float_format(accuracy=1, use_eng_prefix=True)
    print ('Estimated relative error in nominal capacitance values = %.1f%% '
           ' +/-%.1f%%' % (C_relative_error['mean'].mean() * 100,
                           C_relative_error['mean'].std() * 100))
    print C_relative_error[['mean', 'std']] * 100
    print


    return C_relative_error


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


def calculate_stats(df, groupby='test_capacitor'):
    cleaned_df = df.dropna().copy()
    stats = cleaned_df.groupby(groupby)['C'].agg(['mean', 'std', 'median'])
    stats['bias %'] = (cleaned_df.groupby(groupby)
                     .apply(lambda x: ((x['C'] - x['test_capacitor'])).mean() /
                            x['C'].mean())) * 100
    stats['RMSE %'] = 100 * (cleaned_df.groupby(groupby)
                           .apply(lambda x: np.sqrt(((x['C'] -
                                                      x['test_capacitor']) **
                                                     2).mean()) /
                                                    x['C'].mean()))
    stats['cv %'] = stats['std'] / stats['mean'] * 100
    return stats


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


def plot_colormap(stats, column, axis=None):
    freq_vs_C_rmse = stats.reindex_axis(
        pd.Index([(i, j) for i in stats.index.levels[0]
                  for j in stats.index.levels[1]],
                 name=['test_capacitor',
                       'frequency'])).reset_index().pivot(index='frequency',
                                                          columns=
                                                          'test_capacitor',
                                                          values=column)
    if axis is None:
        fig = plt.figure()
        axis = fig.add_subplot(111)
    axis.set_xlabel('Capacitance')
    axis.set_ylabel('Frequency')
    plt.pcolormesh(freq_vs_C_rmse.fillna(1e20).values, vmax=50)
    plt.set_cmap('hot')
    plt.colorbar()
    plt.xticks(np.arange(freq_vs_C_rmse.shape[1])[::2] + 0.5,
               ["%.2fpF" % (c*1e12) for c in freq_vs_C_rmse.columns][::2])
    plt.yticks(np.arange(len(frequencies))[::2] + 0.5,
               ["%.2fkHz" % (f/1e3) for f in frequencies][::2])
    plt.xlim(0, freq_vs_C_rmse.shape[1])
    plt.ylim(0, freq_vs_C_rmse.shape[0])
    plt.setp(plt.xticks()[1], rotation=90)
    plt.tight_layout()
    return axis


def plot_stat_summary(df, fig=None):
    '''
    Plot stats grouped by test capacitor load _and_ frequency.

    In other words, we calculate the mean of all samples in the data
    frame for each test capacitance and frequency pairing, plotting
    the following stats:

     - Root mean squared error
     - Coefficient of variation
     - Bias

    ## [Coefficient of variation][1] ##

    > In probability theory and statistics, the coefficient of
    > variation (CV) is a normalized measure of dispersion of a
    > probability distribution or frequency distribution. It is defined
    > as the ratio of the standard deviation to the mean.

    [1]: http://en.wikipedia.org/wiki/Coefficient_of_variation
    '''
    if fig is None:
        fig = plt.figure(figsize=(8, 8))

    # Define a subplot layout, 3 rows, 2 columns
    grid = GridSpec(3, 2)
    stats = calculate_stats(df, groupby=['test_capacitor',
                                         'frequency'])

    for i, stat in enumerate(['RMSE %', 'cv %', 'bias %']):
        axis = fig.add_subplot(grid[i, 0])
        axis.set_title(stat)
        # Plot a colormap to show how the statistical value changes
        # according to frequency/capacitance pairs.
        plot_colormap(stats, stat, axis=axis)
        axis = fig.add_subplot(grid[i, 1])
        axis.set_title(stat)
        # Plot a histogram to show the distribution of statistical
        # values across all frequency/capacitance pairs.
        stats[stat].hist(bins=50, ax=axis)


if __name__ == '__main__':
    cachedir = path('.cache/2014.10.22/002')

    data = run_experiment('7 caps, 100V, 100Hz-22kHz, rms, with filter (switching board v2.1)')
    #data = fit_fb_calibration(data)
    df = fit_fb_calibration2(data)

    # Write new calibrated feedback resistor and capacitor values to control
    # board configuration.
    import pudb; pudb.set_trace()
    #update_fb_calibration(data['calibration'])
    #estimate_relative_error_in_nominal_capacitance(data)
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
