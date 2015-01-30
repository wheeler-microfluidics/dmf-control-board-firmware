#!/usr/bin/env python
import os
from datetime import datetime
import tempfile
import pkg_resources

from path_helpers import path
import numpy as np
import pandas as pd
import gtk
from dmf_control_board_firmware import DMFControlBoard, Version
import matplotlib
from pygtkhelpers.delegates import WindowView, SlaveView
from pygtkhelpers.ui.form_view_dialog import create_form_view
from flatland.schema import String, Form, Integer, Boolean, Float
from flatland.validation import ValueAtLeast
from IPython.display import display
from dmf_control_board_firmware.calibrate.hv_attenuator import (
    resistor_max_actuation_readings, fit_feedback_params,
    update_control_board_calibration, plot_feedback_params)
from dmf_control_board_firmware.calibrate.oscope import (VISA_AVAILABLE,
                                                         AgilentOscope,
                                                         read_oscope as
                                                         read_oscope_)
from matplotlib.backends.backend_gtkagg import (FigureCanvasGTKAgg as
                                                FigureCanvasGTK)
from matplotlib.backends.backend_gtkagg import (NavigationToolbar2GTKAgg as
                                                NavigationToolbar)
from matplotlib.figure import Figure
if VISA_AVAILABLE:
    import visa


class AssistantView(WindowView):
    def __init__(self, control_board):
        self.control_board = control_board
        self.settings = {}
        self.settings['frequency'] = self.control_board.waveform_frequency()
        self.settings['voltage'] = self.control_board.waveform_voltage()
        self.settings['amplifier_gain'] = self.control_board.amplifier_gain
        self.settings['auto_adjust_amplifier_gain'] = \
                self.control_board.auto_adjust_amplifier_gain
        super(AssistantView, self).__init__(self)

    def restore_settings(self):
        for k in ('amplifier_gain', 'auto_adjust_amplifier_gain'):
            setattr(self.control_board, k, self.settings[k])
        self.control_board.set_waveform_frequency(self.settings['frequency'])
        self.control_board.set_waveform_voltage(self.settings['voltage'])

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
                   'calibrating the high-voltage reference load feedback '
                   'measurement circuit.  This feedback circuit is used to '
                   'measure the output voltage of the amplifier on the control'
                   'board.\n\nSee '
                   r'<a href="http://microfluidics.utoronto.ca/trac/dropbot/wiki/Control board calibration#high-voltage-attenuation-calibration">'
                   'here</a> for more details.')
        label = gtk.Label(content)
        label.set_use_markup(True)
        label.set_line_wrap(True)
        image = gtk.Image()
        img_path = pkg_resources.resource_filename(
            'dmf_control_board_firmware', 'gui/reference_feedback_intro.png')
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
                          ' - Use T-splitter to connect amplifier output to:\n'
                          '   1) DropBot "<tt>In from Amp</tt>".\n'
                          '   2) Oscilloscope input.')
        image = gtk.Image()
        img_path = pkg_resources.resource_filename(
            'dmf_control_board_firmware', 'gui/reference_feedback_setup.png')
        image.set_from_file(str(img_path))
        label.set_line_wrap(True)
        label.set_use_markup(True)
        box.pack_start(label, True, False, padding=15)
        box.pack_start(image, True, True, padding=5)
        self.widget.set_page_complete(box, True)

        # # Select frequencies #
        minimum = self.control_board.min_waveform_frequency
        maximum = self.control_board.max_waveform_frequency
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

        # # Record measurements #
        box1 = gtk.VBox()
        self.widget.append_page(box1)
        self.widget.set_page_type(box1, gtk.ASSISTANT_PAGE_PROGRESS)
        self.widget.set_page_title(box1, "Record measurements")
        self.measurements_label = gtk.Label('Ready')
        self.measurements_label.set_line_wrap(True)
        self.measure_progress = gtk.ProgressBar()
        self.measure_progress.set_size_request(300, 40)
        box1.pack_start(self.measurements_label, True, True, 0)
        box1.pack_start(self.measure_progress, expand=False, fill=False,
                        padding=15)
        self.box1 = box1

        # # Confirm fitted parameters #
        box = gtk.VBox()
        self.widget.append_page(box)
        self.widget.set_page_type(box, gtk.ASSISTANT_PAGE_CONFIRM)
        self.widget.set_page_title(box, "Confirm fitted parameters")
        figure = Figure(figsize=(14, 8), dpi=60)
        self.canvas = FigureCanvasGTK(figure)
        toolbar = NavigationToolbar(self.canvas, self.widget)
        self.axis = figure.add_subplot(111)
        box.pack_start(self.canvas)
        box.pack_start(toolbar, False, False)
        self.widget.set_page_complete(box, True)

        # # Summary #
        box = gtk.VBox()
        self.widget.append_page(box)
        self.widget.set_page_type(box, gtk.ASSISTANT_PAGE_SUMMARY)
        self.widget.set_page_title(box, "Summary")
        label = gtk.Label('Calibration of reference load feedback circuit is '
                          'complete.  The high-voltage output from amplifier '
                          'should now be measured accurately by the control '
                          'board.')
        label.set_line_wrap(True)
        box.pack_start(label, True, True, 0)
        self.widget.set_page_complete(box, True)

    def assistant_prepared(self, assistant, *args):
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
            frequencies = np.logspace(np.log10(start_frequency),
                                      np.log10(end_frequency), number_of_steps)
            self.reset_measurement_count(frequencies)
            gtk.idle_add(self.read_measurements, frequencies)
        elif assistant.get_current_page() == 4:
            self.fit_feedback_params()
            display(self.fitted_params)
            hw_version = Version.fromstring(self.control_board
                                            .hardware_version()).major
            plot_feedback_params(hw_version, self.hv_readings,
                                 self.fitted_params, axis=self.axis)

    @property
    def resistor_count(self):
        return len(self.control_board.a0_series_resistance)
    
    def reset_measurement_count(self, frequencies):
        self.measurement_count = len(frequencies) * self.resistor_count + 2
        self.measurement_i = 0
        self.measurements_label.set_label('Ready.')

    def read_measurements(self, frequencies):
        gtk.gdk.threads_enter()
        self.reset_measurement_count(frequencies)
        gtk.gdk.threads_leave()
        try:
            if VISA_AVAILABLE:
                try:
                    oscope = AgilentOscope()
                    self._read_oscope = lambda: oscope.read_ac_vrms()
                except visa.VisaIOError:
                    self._read_oscope = read_oscope_
            else:
                self._read_oscope = read_oscope_
            self.hv_readings = resistor_max_actuation_readings(
                self.control_board, frequencies, self.read_oscope)
            gtk.gdk.threads_enter()
            self.widget.set_page_complete(self.box1, True)
            gtk.gdk.threads_leave()
        except StopIteration:
            self.measurements_label.set_label('Ready.')
            self.widget.set_current_page(2)
        finally:
            self.restore_settings()

    def read_oscope(self):
        gtk.gdk.threads_enter()
        result = self._read_oscope()
        self.measurement_i += 1
        self.measurements_label.set_label('Measuring amplifier voltage '
                                          'readings...')
        self.measure_progress.set_fraction(float(self.measurement_i) /
                                           self.measurement_count)
        self.measure_progress.set_text('Measurement: %s/%s' %
                                       (self.measurement_i,
                                        self.measurement_count))
        gtk.gdk.threads_leave()
        return result

    def apply_button_clicked(self, assistant):
        # Update the control board with the computed resistive and capacitive
        # load values.  The control board uses these values to compute `V1`
        # using a transfer function when calculating the gain of the amplifier.
        update_control_board_calibration(self.control_board,
                                         self.fitted_params)

    def close_button_clicked(self, assistant):
        print("The 'Close' button has been clicked")
        gtk.main_quit()

    def cancel_button_clicked(self, assistant):
        print("The 'Cancel' button has been clicked")
        gtk.main_quit()

        output_dir = path(tempfile.mkdtemp(prefix='dropbot-reference-calibration'))
        timestamp = datetime.now().strftime('%Y-%m-%dT%Hh%Mm%S')
        self.calibration_file = output_dir.joinpath('%s-calibration.h5' %
                                                    timestamp)

    def fit_feedback_params(self):
        # Using the collected measurements, fit the resistive and *(parasitic)*
        # capacitive load values for the reference *(i.e., high-voltage)*
        # feedback resistor ladder.
        self.fitted_params = fit_feedback_params(self.control_board
                                                 .calibration,
                                                 self.hv_readings)

    def to_hdf(self, output_path, complib='zlib', complevel=6):
        # save control board meta data
        proxy = self.control_board
        df = {}
        df['software_version'] = proxy.software_version()
        df['hardware_version'] = proxy.hardware_version()
        df['serial_number'] = str(proxy.serial_number)
        df['aref'] = str(proxy.__aref__) # need to store as string or to_hdf
                                         # will raise an error
        for address, desc in sorted(proxy._i2c_devices.items()):
            df['i2c address %d' % address] = desc
        pd.Series(df).to_hdf(str(output_path),
                             '/feedback/reference/control_board_info',
                             format='t', complib=complib, complevel=complevel)
        
        # Save measurements taken during calibration, along with input RMS
        # voltage _(i.e., `V1`)_ values read using the oscilloscope.
        self.hv_readings.to_hdf(str(output_path),
                                '/feedback/reference/measurements', format='t',
                                data_columns=self.hv_readings.columns,
                                complib=complib, complevel=complevel)

        # Save fitted resistive and capacitive impedance values.
        self.fitted_params.to_hdf(str(output_path),
                                  '/feedback/reference/fitted_params',
                                  format='t',
                                  data_columns=self.fitted_params.columns,
                                  complib=complib, complevel=complevel)


if __name__ == '__main__':
    control_board = DMFControlBoard()
    control_board.connect()
    view = AssistantView(control_board)
    view.show_and_run()
