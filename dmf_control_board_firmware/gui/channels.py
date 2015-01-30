#!/usr/bin/env python
import pkg_resources
import re

import numpy as np
import pandas as pd
import gtk
from pygtkhelpers.delegates import WindowView
from pygtkhelpers.ui.form_view_dialog import create_form_view
from flatland.schema import Form, Integer, Enum
from flatland.validation import ValueAtLeast, ValueAtMost
from dmf_control_board_firmware import DMFControlBoard
from dmf_control_board_firmware.calibrate.impedance import TEST_LOADS
from matplotlib.backends.backend_gtkagg import (FigureCanvasGTKAgg as
                                                FigureCanvasGTK)
from matplotlib.backends.backend_gtkagg import (NavigationToolbar2GTKAgg as
                                                NavigationToolbar)
from matplotlib.figure import Figure
from datetime import datetime


def sweep_channels(proxy, test_loads, voltage=10, frequency=None,
                   on_update=None):
    if frequency is None:
        # Select frequency in middle of allowed range.
        frequency = 0.5 * (proxy.max_waveform_frequency +
                           proxy.min_waveform_frequency)

    def default_on_update(df):
        item = df.iloc[0]
        print ','.join((k, item[k]) for k in ('channel',
                                              'expected capacitance'))
    if on_update is None:
        on_update = default_on_update

    n_samples = 10

    # Set device frequency and voltage.
    proxy.set_waveform_frequency(frequency)
    proxy.set_waveform_voltage(voltage)
    # Measure impedance to trigger auto-amplifier gain update.
    results = proxy.measure_impedance(5, 100, 0, True, True, [])
    # store the capacitance with all switches off
    C_off = np.max(results.capacitance()[-n_samples:])
    n_channels = proxy.number_of_channels()

    def measure_load(channel, expected_load):
        states = pd.Series(np.zeros(n_channels))
        states[channel] = 1
        results = proxy.measure_impedance(5, n_samples, 0, True, True,
                                          states)
        df = pd.DataFrame(results.capacitance(),
                          columns=['measured capacitance'])
        df['measured impedance'] = results.Z_device()
        df['actuation voltage'] = results.V_actuation()
        df['expected capacitance'] = expected_load
        df['channel'] = channel
        df['RMS error'] = np.abs((df['measured capacitance'] -
                                  df['expected capacitance'])
                                 / df['expected capacitance'])
        df['voltage'] = voltage
        df['frequency'] = frequency
        df['V_hv'] = results.V_hv
        df['V_fb'] = results.V_fb
        df['hv_resistor'] = results.hv_resistor
        df['fb_resistor'] = results.hv_resistor
        df['C_off'] = C_off
        df['amplifier_gain'] = results.amplifier_gain
        df['vgnd_hv'] = results.vgnd_hv
        df['vgnd_fb'] = results.vgnd_fb
        result = df.dropna()
        on_update(result)
        return result

    result_frames = []
    for channel, v in test_loads.iteritems():
        result_frames.append(measure_load(channel, v))

    results = pd.concat(result_frames).groupby('channel').agg('median')

    address = proxy.switching_board_i2c_address + channel / 40
    results['hv_switching_board' % address] = proxy._i2c_devices[address]
    results['software_version'] = proxy.software_version()
    results['hardware_version'] = proxy.hardware_version()
    results['serial_number'] = str(proxy.serial_number)
    results['aref'] = proxy.__aref__
    results['timestamp'] = datetime.now().isoformat()
    
    return results


def plot_channel_sweep(proxy, start_channel):
    test_loads = TEST_LOADS.copy()
    test_loads.index += start_channel

    results = sweep_channels(proxy, test_loads)
    normalized_measurements = (results['measured capacitance']
                               / results['expected capacitance'])
    fig, axis = plt.subplots(figsize=(10, 8))
    axis.bar(normalized_measurements.index - 0.3, normalized_measurements,
             width=0.6, edgecolor='none', facecolor='limegreen')
    axis.set_xlim(left=test_loads.index.min() - 0.5,
                  right=test_loads.index.max() + 0.5)
    axis.set_xlabel('channel')
    axis.set_ylabel(r'$\frac{C_{\tt{measured}}}{C_{\tt{expected}}}$',
                    fontsize=28)
    return results


class AssistantView(WindowView):
    def __init__(self, control_board):
        self.control_board = control_board
        self.channels_per_board = 40
        self.channel_count = self.control_board.number_of_channels()
        self.settings = {}
        self.settings['frequency'] = \
            self.control_board.waveform_frequency()
        self.settings['channel_states'] = \
            self.control_board.state_of_all_channels
        self.settings['voltage'] = self.control_board.waveform_voltage()
        self.settings['amplifier_gain'] = self.control_board.amplifier_gain
        self.settings['auto_adjust_amplifier_gain'] = \
            self.control_board.auto_adjust_amplifier_gain
        self.radio_buttons = []
        self.readings = None
        super(AssistantView, self).__init__(self)

    def restore_settings(self):
        if self.control_board is not None:
            for k in ('amplifier_gain', 'auto_adjust_amplifier_gain'):
                setattr(self.control_board, k, self.settings[k])
            self.control_board.set_waveform_frequency(self
                                                      .settings['frequency'])
            self.control_board.set_waveform_voltage(self.settings['voltage'])
            self.control_board.set_state_of_all_channels(
                self.settings['channel_states'])

    def create_ui(self):
        self.widget = gtk.Assistant()
        self.widget.connect("prepare", self.assistant_prepared)
        self.widget.connect("cancel", self.cancel_button_clicked)
        self.widget.connect("close", self.close_button_clicked)
        self.widget.connect("apply", self.apply_button_clicked)

        # # Introduction #
        box = gtk.HBox()
        self.widget.append_page(box)
        self.widget.set_page_type(box, gtk.ASSISTANT_PAGE_INTRO)
        self.widget.set_page_title(box, "Introduction")
        content = ('This wizard will guide you through the process of '
                   'testing the channels of a single switching board.')
        label = gtk.Label(content)
        label.set_use_markup(True)
        label.set_line_wrap(True)
        image = gtk.Image()
        img_path = pkg_resources.resource_filename(
            'dmf_control_board_firmware', 'gui/channels_intro.png')
        image.set_from_file(str(img_path))
        box.pack_start(label, True, False, padding=15)
        box.pack_start(image, True, True, padding=5)
        self.widget.set_page_complete(box, True)

        # # Connect hardware #
        box = gtk.HBox()
        self.widget.append_page(box)
        self.widget.set_page_type(box, gtk.ASSISTANT_PAGE_CONTENT)
        self.widget.set_page_title(box, "Connect hardware")
        label = gtk.Label(' - Connect DropBot "<tt>Out to Amp</tt>" to amplifier input.\n'
                          ' - Connect amplifier output to DropBot "<tt>In from Amp</tt>".\n'
                          ' - Connect a bank of DropBot channels to test board.')
        image = gtk.Image()
        img_path = pkg_resources.resource_filename(
            'dmf_control_board_firmware', 'gui/impedance_feedback_setup.png')
        image.set_from_file(str(img_path))
        label.set_line_wrap(True)
        label.set_use_markup(True)
        box.pack_start(label, True, False, padding=15)
        box.pack_start(image, True, True, padding=5)
        self.widget.set_page_complete(box, True)

        # # Select frequencies #
        self.radio_buttons = [gtk.RadioButton(label='Channels %d-%d' %
                                              (c, c + self.channels_per_board -
                                               1))
                              for c in xrange(0, self.channel_count,
                                              self.channels_per_board)]

        for b in self.radio_buttons[1:]:
            b.set_group(self.radio_buttons[0])

        box = gtk.VBox()
        for b in self.radio_buttons:
            box.pack_start(b, expand=False, fill=False, padding=20)
        self.widget.append_page(box)
        self.widget.set_page_type(box, gtk.ASSISTANT_PAGE_CONTENT)
        self.widget.set_page_title(box, "Select bank of test channels")
        self.widget.set_page_complete(box, True)

        ## # Record measurements #
        box1 = gtk.VBox()
        self.widget.append_page(box1)
        self.widget.set_page_type(box1, gtk.ASSISTANT_PAGE_PROGRESS)
        self.widget.set_page_title(box1, "Record measurements")
        self.measurements_label = gtk.Label('Ready.')
        self.measurements_label.set_line_wrap(True)
        self.measure_progress = gtk.ProgressBar()
        box1.pack_start(self.measurements_label, True, True, 0)
        box1.pack_start(self.measure_progress, expand=False, fill=False,
                        padding=15)
        self.box1 = box1

        ## # Display readings #
        box = gtk.VBox()
        self.widget.append_page(box)
        self.widget.set_page_type(box, gtk.ASSISTANT_PAGE_CONFIRM)
        self.widget.set_page_title(box, "Channel readings")
        self.figure = Figure(dpi=72)
        self.canvas = FigureCanvasGTK(self.figure)
        toolbar = NavigationToolbar(self.canvas, self.widget)
        box.pack_start(self.canvas)
        box.pack_start(toolbar, False, False)
        self.widget.set_page_complete(box, True)

        ### # Summary #
        box = gtk.VBox()
        self.widget.append_page(box)
        self.widget.set_page_type(box, gtk.ASSISTANT_PAGE_SUMMARY)
        self.widget.set_page_title(box, "Summary")
        label = gtk.Label('Testing of selected channels complete.')
        label.set_line_wrap(True)
        box.pack_start(label, True, True, 0)
        self.widget.set_page_complete(box, True)

    def selected_channels(self):
        return [r for r in self.radio_buttons[0].get_group()
                if r.get_active()][0].get_label()

    def assistant_prepared(self, assistant, *args):
        if assistant.get_current_page() not in (4, 5):
            self.width, self.height = self.widget.size_request()

        if assistant.get_current_page() not in (0, 4):
            print self.width, self.height
            self.widget.resize(self.width, self.height)

        print 'Page %s prepared.' % assistant.get_current_page()
        if assistant.get_current_page() == 3:
            channels_str = self.selected_channels()
            start_channel = int(re.match(r'Channels (?P<start_channel>\d+)-\d+',
                                         channels_str).group('start_channel'))
            self.measurements_label.set_label('Reading measurements (this '
                                              'might take a few minutes)...')
            gtk.idle_add(self.read_measurements, start_channel)
        elif assistant.get_current_page() == 4:
            self.widget.resize(600, 700)
            normalized_measurements = (self.readings['measured capacitance']
                                       / self.readings['expected capacitance'])
            self.figure.clf()
            axis = self.figure.add_subplot(111)
            axis.bar(normalized_measurements.index - 0.3,
                     normalized_measurements, width=0.6, edgecolor='none',
                     facecolor='limegreen')
            axis.set_xlim(left=self.test_loads.index.min() - 0.5,
                          right=self.test_loads.index.max() + 0.5)
            axis.set_xlabel('channel')
            axis.set_ylabel(r'$\frac{C_{\tt{measured}}}{C_{\tt{expected}}}$',
                            fontsize=28)

    def read_measurements(self, start_channel):
        try:
            # No impedance_readings were provided, so run impedance routine to
            # collect measurements.
            def on_update(df):
                gtk.gdk.threads_enter()
                self.measurements_label.set_label('channel=%d' %
                                                  df.iloc[0]['channel'])
                i = int(df.iloc[0]['channel'] % self.channels_per_board)
                self.measure_progress.set_fraction(float(i + 1)
                                                   / self.channels_per_board)
                self.measure_progress.set_text('Measurement: %s/%s' %
                                            (i + 1, self.channels_per_board))
                width, height = self.measure_progress.size_request()
                self.measure_progress.set_size_request(width, 40)
                while gtk.events_pending():
                    gtk.main_iteration(False)
                gtk.gdk.threads_leave()

            self.test_loads = TEST_LOADS.copy()
            self.test_loads.index += start_channel

            self.readings = sweep_channels(self.control_board, self.test_loads,
                                           on_update=on_update)
        finally:
            self.restore_settings()
        gtk.gdk.threads_enter()
        self.measurements_label.set_label('Done.')
        self.widget.set_page_complete(self.box1, True)
        gtk.gdk.threads_leave()

    def apply_button_clicked(self, assistant):
        print("The 'Apply' button has been clicked")

    def close_button_clicked(self, assistant):
        print("The 'Close' button has been clicked")
        gtk.main_quit()

    def cancel_button_clicked(self, assistant):
        print("The 'Cancel' button has been clicked")
        gtk.main_quit()

    def to_hdf(self, output_path, complib='zlib', complevel=6):
        
        def write_results(output_path, df, append=True):
            # Save measurements taken during sweep.
            df.to_hdf(str(output_path), '/channel_sweeps',
                      format='t', append=append,
                      data_columns=self.readings.columns,
                      complib=complib, complevel=complevel)

        try:
            write_results(output_path, self.readings)
        except ValueError:
            df = pd.read_hdf(str(output_path), '/channel_sweeps')
            self.readings = pd.concat([self.readings, df])
            write_results(output_path, self.readings, append=False)

if __name__ == '__main__':
    control_board = DMFControlBoard()
    control_board.connect()
    view = AssistantView(control_board)
    view.show_and_run()
    if view.readings is not None:
        view.to_hdf('output.h5')
