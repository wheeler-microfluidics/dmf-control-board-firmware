# coding: utf-8
r'''
This module contains code to plot the capacitance of each channel on a digital
microfluidics chip.  The plots are useful, for example, to identify potential
open circuit connections (e.g., broken traces).
'''
from matplotlib.colors import Normalize
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import FuncFormatter
from matplotlib.transforms import offset_copy
from si_prefix import si_format
from svg_model.plot import plot_shapes_heat_map, plot_color_map_bars
import matplotlib.cm as mcm
import matplotlib.pyplot as plt
import pandas as pd


F_formatter = FuncFormatter(lambda x, pos: '%sF' % si_format(x))
m_formatter = FuncFormatter(lambda x, pos: '%sm' % si_format(x, 0))


def plot_channel_capacitance(channel_capacitance, vmax=(200e-15),
                             color_map=mcm.Reds_r, **kwargs):
    vmax = max(2 * channel_capacitance.min(), vmax)
    axis = plot_color_map_bars(channel_capacitance, color_map=color_map,
                               vmax=vmax, **kwargs)
    axis.yaxis.set_major_formatter(F_formatter)
    return axis


def plot_electrode_capacitance(df_shapes, channel_capacitance,
                               electrodes_by_channel, vmax=(200e-15),
                               color_map=mcm.Reds_r, **kwargs):
    vmax = max(2 * channel_capacitance.min(), vmax)
    electrode_ids = electrodes_by_channel.ix[channel_capacitance.index]
    electrode_capacitance = pd.Series(channel_capacitance.ix
                                      [electrode_ids.index].values,
                                      index=electrode_ids.values)

    df_shapes = df_shapes.copy()

    # Scale millimeters to meters.
    df_shapes[['x', 'y']] *= 1e-3

    axis, colorbar = plot_shapes_heat_map(df_shapes, 'id',
                                          electrode_capacitance,
                                          value_formatter=F_formatter,
                                          vmax=vmax, color_map=color_map,
                                          **kwargs)

    axis.xaxis.set_major_formatter(m_formatter)
    map(lambda t: t.set_rotation(90), axis.get_xticklabels())
    axis.yaxis.set_major_formatter(m_formatter)
    axis.set_aspect(True)
    return axis, colorbar


def plot_capacitance_summary(data, fig=None, color_map=mcm.Reds_r,
                             vmax=200e-15,  # 200fF
                             reduce_func='median'):
    '''
    | ---------- | ------------------------- |
    |            |      Capacitance of       |
    |   Device   |   channels (index order)  |
    |   drawing  | ------------------------- |
    |            |      Capacitance of       |
    |            |    channels (C order)     |
    | ---------- | ------------------------- |
    '''
    # Get median capacitance reading for each channel.
    channel_groups = data['channel_impedances'].groupby('channel_i')
    channel_capacitance = getattr(channel_groups['capacitance'], reduce_func)()
    vmax = max(.5 * (channel_capacitance.median() + channel_capacitance.min()),
               vmax)

    grid = GridSpec(2, 8)
    if fig is None:
        fig = plt.figure(figsize=(25, 10))

    axes = [fig.add_subplot(grid[:, :3]),
            fig.add_subplot(grid[0, 3:]),
            fig.add_subplot(grid[1, 3:])]


    def label_electrodes(axis, df_shapes, channels_by_electrode):
        df_shape_min = df_shapes.groupby('id')[['x', 'y']].min() * 1e-3
        df_shape_max = df_shapes.groupby('id')[['x', 'y']].max() * 1e-3
        df_shape_centers = .5 * (df_shape_max + df_shape_min)
        df_shape_centers.y = df_shapes.y.max() * 1e-3 - df_shape_centers.y

        light_color = '#ffffff'
        dark_color = '#000000'

        values = channel_capacitance
        norm = Normalize(min(values), vmax, clip=True)
        colors = color_map(norm(values.values).filled())
        lightness = pd.Series(colors[:, :3].mean(axis=1), index=values.index)

        for electrode_i, (x_i, y_i) in df_shape_centers.iterrows():
            channel_i = channels_by_electrode.ix[electrode_i]
            axis.text(x_i, y_i, channel_i, horizontalalignment='center',
                    verticalalignment='center',
                    color=dark_color if channel_i in lightness.index and
                    lightness.ix[channel_i] > 0.5 else light_color)

    plot_electrode_capacitance(data['shapes'],
                               channel_capacitance,
                               data['device/electrodes_by_channel'],
                               axis=axes[0], vmax=vmax)
    # Label channel(s) for each electrode on heat map.
    label_electrodes(axes[0], data['shapes'],
                     data['device/channels_by_electrode'])

    # Plot channel capacitances as 2 bar charts, colored according to heat map.
    # -------------------------------------------------------------------------

    # The x-axis of the first bar chart is ordered by channel number.
    plot_color_map_bars(channel_capacitance, color_map=color_map, axis=axes[1],
                        vmax=vmax)
    channel_capacitance_ordered = channel_capacitance.sort_values()

    # The x-axis of the second bar chart is ordered by channel capacitance.
    plot_color_map_bars(channel_capacitance_ordered, color_map=color_map,
                        axis=axes[2], vmax=vmax)

    def label_bars(axis, values, fontsize=8, **kwargs):
        # Offset labels by 10% of the axis height.
        trans_offset = offset_copy(axis.transData, fig=axis.get_figure(),
                                   y=0.10)
        for i, value_i in zip(axis.get_xticks(), values):
            axis.text(i, value_i, F_formatter(value_i),
                      horizontalalignment='center', verticalalignment='bottom',
                      rotation=90, fontsize=fontsize, transform=trans_offset)

    # Annotate capacitance bars with capacitance value.
    label_bars(axes[1], channel_capacitance)
    label_bars(axes[2], channel_capacitance_ordered)

    for ax in axes:
        ax.yaxis.set_major_formatter(F_formatter)
    fig.tight_layout()
    return axes
