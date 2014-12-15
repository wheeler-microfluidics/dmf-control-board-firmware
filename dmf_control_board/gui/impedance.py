#!/usr/bin/env python
import os
from datetime import datetime
import tempfile
import pkg_resources

from path_helpers import path
import numpy as np
import pandas as pd
import gtk
from dmf_control_board import DMFControlBoard, Version
from pygtkhelpers.delegates import WindowView, SlaveView
from pygtkhelpers.ui.form_view_dialog import create_form_view
from flatland.schema import String, Form, Integer, Boolean, Float
from flatland.validation import ValueAtLeast
from IPython.display import display
from dmf_control_board.calibrate.impedance import (run_experiment,
                                                   fit_fb_calibration,
                                                   apply_calibration,
                                                   update_fb_calibration)
from dmf_control_board.calibrate.impedance_benchmarks import (
    plot_stat_summary)
from matplotlib.backends.backend_gtkagg import (FigureCanvasGTKAgg as
                                                FigureCanvasGTK)
from matplotlib.backends.backend_gtkagg import (NavigationToolbar2GTKAgg as
                                                NavigationToolbar)
from matplotlib.figure import Figure


class AssistantView(WindowView):
    def __init__(self, control_board, impedance_readings=None,
                 calibration=None):
        self.control_board = control_board
        self.settings = {}
        if control_board is not None:
            self.settings['frequency'] = \
                self.control_board.waveform_frequency()
            self.settings['channel_states'] = \
                self.control_board.state_of_all_channels
            self.settings['voltage'] = self.control_board.waveform_voltage()
            self.settings['amplifier_gain'] = self.control_board.amplifier_gain
            self.settings['auto_adjust_amplifier_gain'] = \
                self.control_board.auto_adjust_amplifier_gain
        if calibration is None:
            self.calibration = self.control_board.calibration
        else:
            self.calibration = calibration
        self.impedance_readings = impedance_readings
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
                   'calibrating the device load feedback measurement circuit. '
                   'This feedback circuit is used to measure the impedance '
                   'between the actuated area and ground.  This impedance is '
                   'related to the volume of liquid between the actuated area'
                   '\n\nSee '
                   r'<a href="http://microfluidics.utoronto.ca/trac/dropbot/wiki/Control board calibration#device-load-impedance-calibration">'
                   'here</a> for more details.')
        label = gtk.Label(content)
        label.set_use_markup(True)
        label.set_line_wrap(True)
        image = gtk.Image()
        img_path = pkg_resources.resource_filename(
            'dmf_control_board', 'gui/impedance_feedback_intro.png')
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
                          ' - Connect DropBot "<tt>0-39</tt>" to test board.')
        image = gtk.Image()
        img_path = pkg_resources.resource_filename(
            'dmf_control_board', 'gui/impedance_feedback_setup.png')
        image.set_from_file(str(img_path))
        label.set_line_wrap(True)
        label.set_use_markup(True)
        box.pack_start(label, True, False, padding=15)
        box.pack_start(image, True, True, padding=5)
        self.widget.set_page_complete(box, True)

        # # Select frequencies #
        #minimum = self.control_board.min_waveform_frequency
        #maximum = self.control_board.max_waveform_frequency
        minimum = 100
        maximum = 20e3
        form = Form.of(
            Integer.named('start_frequency').using(
                default=minimum, optional=True,
                validators=[ValueAtLeast(minimum=minimum), ]),
            Integer.named('end_frequency').using(
                default=maximum, optional=True,
                validators=[ValueAtLeast(minimum=minimum), ]),
            Integer.named('number_of_steps').using(
                default=10, optional=True,
                validators=[ValueAtLeast(minimum=2), ]),
        )
        box = gtk.HBox()
        self.form_view = create_form_view(form)
        self.form_view.form.proxies.connect('changed', display)
        box.pack_start(self.form_view.widget, fill=False, padding=40)
        self.widget.append_page(box)
        self.widget.set_page_type(box, gtk.ASSISTANT_PAGE_CONTENT)
        self.widget.set_page_title(box, "Select calibration frequencies")
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

        # # Confirm fitted parameters #
        box = gtk.VBox()
        self.widget.append_page(box)
        self.widget.set_page_type(box, gtk.ASSISTANT_PAGE_CONFIRM)
        self.widget.set_page_title(box, "Confirm fitted parameters")
        self.figure = Figure(dpi=72)
        self.canvas = FigureCanvasGTK(self.figure)
        toolbar = NavigationToolbar(self.canvas, self.widget)
        #self.axis = figure.add_subplot(111)
        box.pack_start(self.canvas)
        box.pack_start(toolbar, False, False)
        self.widget.set_page_complete(box, True)

        ## # Summary #
        box = gtk.VBox()
        self.widget.append_page(box)
        self.widget.set_page_type(box, gtk.ASSISTANT_PAGE_SUMMARY)
        self.widget.set_page_title(box, "Summary")
        label = gtk.Label('Calibration of device load feedback circuit is '
                          'complete.  The impedance between actuated device '
                          'area and ground should now be measured accurately '
                          'by the control board.')
        label.set_line_wrap(True)
        box.pack_start(label, True, True, 0)
        self.widget.set_page_complete(box, True)

    def assistant_prepared(self, assistant, *args):
        if assistant.get_current_page() not in (4, 5):
            self.width, self.height = self.widget.size_request()

        if assistant.get_current_page() not in (0, 4):
            print self.width, self.height
            self.widget.resize(self.width, self.height)

        print 'Page %s prepared.' % assistant.get_current_page()
        if assistant.get_current_page() < 3:
            self.widget.set_page_complete(self.box1, False)

        if assistant.get_current_page() == 3:
            settings = dict([(f, self.form_view.form.fields[f].proxy
                              .get_widget_value())
                             for f in ('start_frequency', 'number_of_steps',
                                       'end_frequency')])
            start_frequency = np.array(settings['start_frequency'])
            end_frequency = np.array(settings['end_frequency'])
            number_of_steps = np.array(settings['number_of_steps'])
            frequencies = np.floor(np.logspace(np.log10(start_frequency),
                                               np.log10(end_frequency),
                                               number_of_steps))
            self.measurements_label.set_label('Reading measurements (this '
                                              'might take a few minutes)...')
            gtk.idle_add(self.read_measurements, frequencies)
        elif assistant.get_current_page() == 4:
            self.widget.resize(600, 700)
            display(self.fitted_params)
            fitted_readings = self.impedance_readings.copy()
            apply_calibration(fitted_readings, self.fitted_params,
                              self.calibration)
            plot_stat_summary(fitted_readings, fig=self.figure)

    def read_measurements(self, frequencies):
        if self.impedance_readings is None:
            try:
                # No impedance_readings were provided, so run impedance routine to
                # collect measurements.
                def on_update(frequency, C1, channel, i, group_count, data):
                    gtk.gdk.threads_enter()
                    self.measurements_label.set_label('Frequency=%.2fkHz, '
                                                    'C=%.2fpF' %
                                                    (frequency, C1 * 1e12))
                    self.measure_progress.set_fraction(float(i) / group_count)
                    self.measure_progress.set_text('Measurement: %s/%s' %
                                                (i + 1, group_count))
                    width, height = self.measure_progress.size_request()
                    self.measure_progress.set_size_request(width, 40)
                    while gtk.events_pending():
                        gtk.main_iteration(False)
                    gtk.gdk.threads_leave()
                self.impedance_readings = run_experiment(
                    self.control_board, frequencies=frequencies,
                    on_update=on_update)
            finally:
                self.restore_settings()
        self.fit_feedback_params()
        gtk.gdk.threads_enter()
        self.measurements_label.set_label('Done.')
        self.widget.set_page_complete(self.box1, True)
        gtk.gdk.threads_leave()

    def read_oscope(self):
        gtk.gdk.threads_enter()
        result = self._read_oscope()
        self.measurement_i += 1
        self.measurements_label.set_label('Measurements taken: %d / %d' %
                                          (self.measurement_i,
                                           self.measurement_count))
        gtk.gdk.threads_leave()
        return result

    def apply_button_clicked(self, assistant):
        # Update the control board with the computed resistive and capacitive
        # load values.  The control board uses these values to compute `V1`
        # using a transfer function when calculating the gain of the amplifier.
        update_fb_calibration(self.control_board, self.calibration)

    def close_button_clicked(self, assistant):
        print("The 'Close' button has been clicked")
        gtk.main_quit()

    def cancel_button_clicked(self, assistant):
        print("The 'Cancel' button has been clicked")
        gtk.main_quit()

    def fit_feedback_params(self):
        # Using the collected measurements, fit the resistive and *(parasitic)*
        # capacitive load values for the reference *(i.e., high-voltage)*
        # feedback resistor ladder.
        self.fitted_params = fit_fb_calibration(self.impedance_readings,
                                                self.calibration)

    def to_hdf(self, output_path, complib='blosc', complevel=6):
        # Save measurements taken during calibration.
        self.impedance_readings.to_hdf(str(output_path),
                                       '/feedback/impedance/measurements',
                                       format='t',
                                       data_columns=self.impedance_readings
                                       .columns, complib=complib,
                                       complevel=complevel)

        # Save measurements taken during calibration, along with input RMS
        # voltage _(i.e., `V1`)_ values read using the oscilloscope.
        pd.Series([self.calibration]).to_hdf(str(output_path),
                                             '/feedback/impedance/calibration')

        # Save fitted resistive and capacitive impedance values.
        self.fitted_params.to_hdf(str(output_path),
                                  '/feedback/impedance/fitted_params',
                                  format='t',
                                  data_columns=self.fitted_params.columns,
                                  complib=complib, complevel=complevel)


if __name__ == '__main__':
    if False:
        impedance_readings = pd.read_hdf('/tmp/dropbot-impedance-calibration3NXvK0/2014-12-05T11h21m44-calibration.h5',
                                        '/feedback/impedance/measurements')
        calibration = pd.read_hdf('../../../dropbot-calibration/dropbot_calibration-002-2014-11-18.h5',
                                  '/impedance/calibration')[0]
        view = AssistantView(None, impedance_readings=impedance_readings,
                             calibration=calibration)
    else:
        control_board = DMFControlBoard()
        control_board.connect()
        view = AssistantView(control_board)
    view.show_and_run()
