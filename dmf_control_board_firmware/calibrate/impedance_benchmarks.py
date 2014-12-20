# coding: utf-8
import pandas as pd
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from matplotlib.colors import Colormap
from matplotlib.gridspec import GridSpec
import numpy as np

pd.set_option('display.width', 300)


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
    f = np.tile(np.reshape(frequencies,
                           [len(frequencies)] + [1]*(len(C.shape) - 1)),
                [1] + list(C.shape[1:]))

    # Plot the impedance of each experiment vs frequency (with the data points
    # color-coded according to the feedback resistor).
    # Note that impedance, $Z$, can be computed as:
    #
    #             1
    #     Z = ──────────
    #         2⋅π⋅freq⋅C
    #
    plt.figure(figsize=figsize)
    legend = []
    for i in range(len(calibration.R_fb)):
        legend.append("R$_{fb,%d}$" % i)
        ind = mlab.find(fb_resistor == i)
        plt.loglog(f.flatten()[ind], 1.0 / (2 * np.pi * f.flatten()[ind] *
                                            C.flatten()[ind]), 'o')
        plt.xlim(0.8 * np.min(frequencies), 1.2 * np.max(frequencies))
    for C_device in test_loads:
        # TODO: What is the reason for the `np.ones` below?
        plt.plot(frequencies, 1.0 / (2 * np.pi * C_device *
                                     np.ones(len(frequencies)) * frequencies),
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
                       .apply(lambda x: ((x['C'] - x['test_capacitor'])).mean()
                              / x['C'].mean())) * 100
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


def plot_colormap(stats, column, axis=None, fig=None):
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
    frequencies = stats.index.levels[1]
    axis.set_xlabel('Capacitance')
    axis.set_ylabel('Frequency')
    vmin = freq_vs_C_rmse.fillna(0).values.min()
    vmax = freq_vs_C_rmse.fillna(0).values.max()
    if vmin < 0:
        vmax = np.abs([vmin, vmax]).max()
        vmin = -vmax
        cmap=plt.cm.coolwarm
    else:
        vmin = 0
        cmap=plt.cm.Reds
    mesh = axis.pcolormesh(freq_vs_C_rmse.fillna(0).values, vmin=vmin,
                           vmax=vmax, cmap=cmap)
    if fig is not None:
        fig.colorbar(mesh)
    else:
        plt.colorbar()
    axis.set_xticks(np.arange(freq_vs_C_rmse.shape[1]) + 0.5)
    axis.set_xticklabels(["%.1fpF" % (c*1e12)
                          for c in freq_vs_C_rmse.columns],
                         rotation=90)
    axis.set_yticks(np.arange(len(frequencies)) + 0.5)
    axis.set_yticklabels(["%.2fkHz" % (f / 1e3) for f in frequencies])
    axis.set_xlim(0, freq_vs_C_rmse.shape[1])
    axis.set_ylim(0, freq_vs_C_rmse.shape[0])
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
                                         'frequency']).dropna()

    for i, stat in enumerate(['RMSE %', 'cv %', 'bias %']):
        axis = fig.add_subplot(grid[i, 0])
        axis.set_title(stat)
        # Plot a colormap to show how the statistical value changes
        # according to frequency/capacitance pairs.
        plot_colormap(stats, stat, axis=axis, fig=fig)
        axis = fig.add_subplot(grid[i, 1])
        axis.set_title(stat)
        # Plot a histogram to show the distribution of statistical
        # values across all frequency/capacitance pairs.
        try:
            axis.hist(stats[stat].values, bins=50)
        except AttributeError:
            print stats[stat].describe()
    fig.tight_layout()
