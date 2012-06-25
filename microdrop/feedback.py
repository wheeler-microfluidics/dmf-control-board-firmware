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
import math
import time
from copy import deepcopy
try:
    import cPickle as pickle
except ImportError:
    import pickle

import logging
import gtk
import numpy as np
import matplotlib
import matplotlib.mlab as mlab
if os.name=='nt':
    matplotlib.rc('font', **{'family':'sans-serif','sans-serif':['Arial']})
from matplotlib.figure import Figure
from path import path
import scipy.optimize as optimize
import yaml

try:
    from matplotlib.backends.backend_gtkagg import FigureCanvasGTKAgg as FigureCanvasGTK
    from matplotlib.backends.backend_gtkagg import NavigationToolbar2GTKAgg as NavigationToolbar
except RuntimeError:
    if PROGRAM_LAUNCHED:
        raise
    else:
        logging.info('Skipping error!')

try:
    from ...dmf_control_board import *
except:
    # Raise the exception(s) if we're running the program (these exceptions
    # are expected when generating documentation with doxygen, so in that case
    # we can safely ignore them).
    if utility.PROGRAM_LAUNCHED:
        raise
from utility import SetOfInts, Version, VersionError, FutureVersionError, \
    is_float
from utility.gui import textentry_validate, combobox_set_model_from_list, \
    combobox_get_active_text, text_entry_dialog, FormViewDialog, yesno
from flatland.schema import String, Form, Integer, Boolean, Float
from flatland.validation import ValueAtLeast, ValueAtMost
from plugin_manager import emit_signal, IWaveformGenerator, IPlugin
from app_context import get_app


class AmplifierGainNotCalibrated(Exception):
    pass


def feedback_signal(p, frequency, Z):
    """p[0]=C, p[1]=R"""
    return np.abs(p[1]/(Z+p[1]+Z*p[1]*2*np.pi*p[0]*complex(0,1)*frequency))


class RetryAction():
    class_version = str(Version(0,1))
    capacitance_threshold = 0

    def __init__(self,
                 percent_threshold=None,
                 increase_voltage=None,
                 max_repeats=None):
        if percent_threshold:
            self.percent_threshold = percent_threshold
        else:
            self.percent_threshold = 0
        if increase_voltage:
            self.increase_voltage = increase_voltage
        else:
            self.increase_voltage = 0
        if max_repeats:
            self.max_repeats = max_repeats
        else:
            self.max_repeats = 3
        self.version = self.class_version

    def __setstate__(self, dict):
        self.__dict__ = dict
        if 'version' not in self.__dict__:
            self.version = str(Version(0,0))
        self._upgrade()

    def _upgrade(self):
        """
        Upgrade the serialized object if necessary.

        Raises:
            FutureVersionError: file was written by a future version of the
                software.
        """
        logging.debug("[RetryAction]._upgrade()")
        version = Version.fromstring(self.version)
        logging.debug('[RetryAction] version=%s, class_version=%s' % \
                     (str(version), self.class_version))
        if version > Version.fromstring(self.class_version):
            logging.debug('[RetryAction] version>class_version')
            raise FutureVersionError(Version.fromstring(self.class_version), version)
        elif version < Version.fromstring(self.class_version):
            if version < Version(0,1):
                del self.capacitance_threshold
                self.percent_threshold = 0
                self.version = str(Version(0,1))
                logging.info('[RetryAction] upgrade to version %s' % self.version)
        # else the versions are equal and don't need to be upgraded


class SweepFrequencyAction():
    def __init__(self,
                 start_frequency=None,
                 end_frequency=None,
                 n_frequency_steps=None):
        if start_frequency:
            self.start_frequency = start_frequency
        else:
            self.start_frequency = 1e2
        if end_frequency:
            self.end_frequency = end_frequency
        else:
            self.end_frequency = 30e3
        if n_frequency_steps:
            self.n_frequency_steps = n_frequency_steps
        else:
            self.n_frequency_steps = 30


class SweepVoltageAction():
    def __init__(self,
                 start_voltage=None,
                 end_voltage=None,
                 n_voltage_steps=None):
        if start_voltage:
            self.start_voltage = start_voltage
        else:
            self.start_voltage = 5
        if end_voltage:
            self.end_voltage = end_voltage
        else:
            self.end_voltage = 100
        if n_voltage_steps:
            self.n_voltage_steps = n_voltage_steps
        else:
            self.n_voltage_steps = 20


class SweepElectrodesAction():
    def __init__(self,
                 channels=None):
        if channels:
            self.channels = channels
        else:
            self.channels = SetOfInts()
            app = get_app()
            for e in app.dmf_device.electrodes.values():
                self.channels.update(e.channels)

class FeedbackOptions():
    """
    This class stores the feedback options for a single step in the protocol.
    """
    class_version = str(Version(0,1))
        
    def __init__(self, feedback_enabled=None,
                 action=None):
        if feedback_enabled:
            self.feedback_enabled = feedback_enabled
        else:
            self.feedback_enabled = True
        if action:
            self.action = action
        else:
            self.action = RetryAction()
        self.version = self.class_version

    def _upgrade(self):
        """
        Upgrade the serialized object if necessary.

        Raises:
            FutureVersionError: file was written by a future version of the
                software.
        """
        logging.debug('[FeedbackOptions]._upgrade()')
        if hasattr(self, 'version'):
            version = Version.fromstring(self.version)
        else:
            version = Version(0)
        logging.debug('[FeedbackOptions] version=%s, class_version=%s' % \
                     (str(version), self.class_version))
        if version > Version.fromstring(self.class_version):
            logging.debug('[FeedbackOptions] version>class_version')
            raise FutureVersionError(Version.fromstring(self.class_version),
                                     version)
        elif version < Version.fromstring(self.class_version):
            if version < Version(0,1):
                del self.sampling_time_ms
                del self.n_samples
                del self.delay_between_samples_ms
            self.version = self.class_version
        # else the versions are equal and don't need to be upgraded


class FeedbackOptionsController():
    def __init__(self, plugin):
        self.plugin = plugin
        self.builder = gtk.Builder()
        app = get_app()
        self.builder.add_from_file(
            path(app.config['plugins']['directory']).joinpath('dmf_control_board',
                        'microdrop', 'glade', 'feedback_options.glade'))
        self.window = self.builder.get_object("window")
        self.builder.connect_signals(self)
        self.window.set_title("Feedback Options")
        self.initialized = False

    def on_plugin_enable(self):
        if not self.initialized:
            app = get_app()
            menu_item = gtk.MenuItem("Feedback Options")
            self.plugin.control_board_menu.append(menu_item)
            menu_item.connect("activate", self.on_window_show)
            menu_item.show()
            
            self.measure_cap_menu_item = gtk.MenuItem(
                    "Measure device capacitance")
            app.dmf_device_controller.view.popup.add_item(
                    self.measure_cap_menu_item)
            self.measure_cap_menu_item.connect("activate",
                    self.on_measure_device_capacitance)
            self.initialized = True
        self.measure_cap_menu_item.show()

    def on_plugin_disable(self):
        self.measure_cap_menu_item.hide()

    def on_window_show(self, widget, data=None):
        """
        Handler called when the user clicks on "Feedback Options" in the "Tools"
        menu.
        """
        options = self.plugin.get_step_options().feedback_options
        self._set_gui_sensitive(options)
        self._update_gui_state(options)        
        self.window.show()

    def on_window_delete_event(self, widget, data=None):
        """
        Handler called when the user closes the "Feedback Options" window. 
        """
        self.window.hide()
        return True

    def on_measure_device_capacitance(self, widget, data=None):
        app = get_app()
        if self.plugin.control_board.connected():
            electrode = \
                app.dmf_device_controller.view.popup.last_electrode_clicked
            area = electrode.area() * app.dmf_device.scale
            current_state = self.plugin.control_board.state_of_all_channels
            state = np.zeros(len(current_state))

            if self.plugin.control_board.number_of_channels() < \
                max(electrode.channels):
                logging.warning("Error: "
                    "currently connected board does not have enough channels "
                    "to perform calibration on this electrode.")
                return

            state[electrode.channels]=1
            step = app.protocol.current_step()
            dmf_options = step.get_data(self.plugin.name)
            voltage = dmf_options.voltage
            frequency = dmf_options.frequency
            emit_signal("set_frequency", frequency,
                        interface=IWaveformGenerator)
            emit_signal("set_voltage", voltage, interface=IWaveformGenerator)
            app_values = self.plugin.get_app_values()
            test_options = deepcopy(dmf_options)
            test_options.duration = 10*app_values['sampling_time_ms']
            test_options.feedback_options = FeedbackOptions(
                feedback_enabled=True, action=RetryAction())
            self.plugin.check_impedance(test_options)
            (V_hv, hv_resistor, V_fb, fb_resistor) = \
                self.plugin.measure_impedance(state, test_options,
                    app_values['sampling_time_ms'],
                    app_values['delay_between_samples_ms'])
            results = FeedbackResults(test_options,
                app_values['sampling_time_ms'],
                app_values['delay_between_samples_ms'],
                V_hv, hv_resistor,
                V_fb, fb_resistor,
                area,
                self.plugin.control_board.calibration)
            logging.info('max(results.capacitance())/area=%s' % (max(results.capacitance()) / area))
            self.plugin.control_board.state_of_all_channels = current_state
            RetryAction.capacitance_threshold =\
                max(results.capacitance()) / area

    def on_button_feedback_enabled_toggled(self, widget, data=None):
        """
        Handler called when the "Feedback enabled" check box is toggled. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        options.feedback_enabled = widget.get_active()
        emit_signal('on_step_options_changed',
                    [self.plugin.name, app.protocol.current_step_number],
                    interface=IPlugin)

    def on_step_options_changed(self, plugin_name, step_number):
        app = get_app()
        if self.plugin.name == plugin_name \
                and app.protocol.current_step_number == step_number:
            all_options = self.plugin.get_step_options(step_number)
            options = all_options.feedback_options
            self._set_gui_sensitive(options)
            self._update_gui_state(options)

    def _update_gui_state(self, options):
        # update the state of the "Feedback enabled" check button        
        button = self.builder.get_object("button_feedback_enabled")
        if options.feedback_enabled != button.get_active():
            # Temporarily disable radio-button toggled signal handler to avoid
            # infinite loop (handler emits signal that results in this method
            # being called).
            button.handler_block_by_func(self.on_button_feedback_enabled_toggled)
            button.set_active(options.feedback_enabled)
            button.handler_unblock_by_func(self.on_button_feedback_enabled_toggled)

        # update the retry action parameters
        retry = (options.action.__class__ == RetryAction)
        if retry:
            self.builder.get_object("textentry_percent_threshold"). \
                set_text(str(options.action.percent_threshold))
            self.builder.get_object("textentry_increase_voltage"). \
                set_text(str(options.action.increase_voltage))
            self.builder.get_object("textentry_max_repeats").set_text(
                str(options.action.max_repeats))
        else:
            self.builder.get_object("textentry_percent_threshold"). \
                set_text("")
            self.builder.get_object("textentry_increase_voltage").set_text("")
            self.builder.get_object("textentry_max_repeats").set_text("")
        button = self.builder.get_object("radiobutton_retry")
        if retry != button.get_active():
            # Temporarily disable toggled signal handler (see above)
            button.handler_block_by_func(self.on_radiobutton_retry_toggled)
            button.set_active(retry)
            button.handler_unblock_by_func(self.on_radiobutton_retry_toggled)

        sweep_frequency = (options.action.__class__ == SweepFrequencyAction)
        # update the sweep frequency action parameters
        if sweep_frequency:
            self.builder.get_object("textentry_start_frequency"). \
                set_text(str(options.action.start_frequency/1000.0))
            self.builder.get_object("textentry_end_frequency").set_text(
                str(options.action.end_frequency/1000.0))
            self.builder.get_object("textentry_n_frequency_steps").set_text(
                str(str(options.action.n_frequency_steps)))
        else:
            self.builder.get_object("textentry_start_frequency").set_text("")
            self.builder.get_object("textentry_end_frequency").set_text("")
            self.builder.get_object("textentry_n_frequency_steps").set_text("")
        button = self.builder.get_object("radiobutton_sweep_frequency")
        if sweep_frequency != button.get_active():
            # Temporarily disable toggled signal handler (see above)
            button.handler_block_by_func(self.on_radiobutton_sweep_frequency_toggled)
            button.set_active(sweep_frequency)
            button.handler_unblock_by_func(self.on_radiobutton_sweep_frequency_toggled)

        sweep_voltage = (options.action.__class__ == SweepVoltageAction)
        # update the sweep voltage action parameters
        if sweep_voltage:
            self.builder.get_object("textentry_start_voltage"). \
                set_text(str(options.action.start_voltage))
            self.builder.get_object("textentry_end_voltage").set_text(
                str(options.action.end_voltage))
            self.builder.get_object("textentry_n_voltage_steps").set_text(
                str(str(options.action.n_voltage_steps)))
        else:
            self.builder.get_object("textentry_start_voltage").set_text("")
            self.builder.get_object("textentry_end_voltage").set_text("")
            self.builder.get_object("textentry_n_voltage_steps").set_text("")
        button = self.builder.get_object("radiobutton_sweep_voltage")
        if sweep_voltage != button.get_active():
            # Temporarily disable toggled signal handler (see above)
            button.handler_block_by_func(self.on_radiobutton_sweep_voltage_toggled)
            button.set_active(sweep_voltage)
            button.handler_unblock_by_func(self.on_radiobutton_sweep_voltage_toggled)

        sweep_electrodes = (options.action.__class__ == SweepElectrodesAction)
        # update the sweep electrodes action parameters
        if sweep_electrodes:
            self.builder.get_object("textentry_channels"). \
                set_text(str(options.action.channels))
        else:
            self.builder.get_object("textentry_channels").set_text("")
        button = self.builder.get_object("radiobutton_sweep_electrodes")
        if sweep_electrodes != button.get_active():
            # Temporarily disable toggled signal handler (see above)
            button.handler_block_by_func(self.on_radiobutton_sweep_electrodes_toggled)
            button.set_active(sweep_electrodes)
            button.handler_unblock_by_func(self.on_radiobutton_sweep_electrodes_toggled)

    def _set_gui_sensitive(self, options):
        self.builder.get_object("radiobutton_retry")\
            .set_sensitive(options.feedback_enabled)
        self.builder.get_object("radiobutton_sweep_frequency")\
            .set_sensitive(options.feedback_enabled)
        self.builder.get_object("radiobutton_sweep_voltage")\
            .set_sensitive(options.feedback_enabled)
        self.builder.get_object("radiobutton_sweep_electrodes")\
            .set_sensitive(options.feedback_enabled)

        retry = (options.action.__class__ == RetryAction)
        self.builder.get_object("textentry_percent_threshold")\
            .set_sensitive(options.feedback_enabled and retry)
        self.builder.get_object("textentry_increase_voltage")\
            .set_sensitive(options.feedback_enabled and retry)
        self.builder.get_object("textentry_max_repeats")\
            .set_sensitive(options.feedback_enabled and retry)

        sweep_frequency = (options.action.__class__ == SweepFrequencyAction)
        self.builder.get_object("textentry_start_frequency")\
            .set_sensitive(options.feedback_enabled and sweep_frequency)
        self.builder.get_object("textentry_end_frequency")\
            .set_sensitive(options.feedback_enabled and sweep_frequency)
        self.builder.get_object("textentry_n_frequency_steps")\
            .set_sensitive(options.feedback_enabled and sweep_frequency)

        sweep_voltage = (options.action.__class__ == SweepVoltageAction)
        self.builder.get_object("textentry_start_voltage")\
            .set_sensitive(options.feedback_enabled and sweep_voltage)
        self.builder.get_object("textentry_end_voltage")\
            .set_sensitive(options.feedback_enabled and sweep_voltage)
        self.builder.get_object("textentry_n_voltage_steps")\
            .set_sensitive(options.feedback_enabled and sweep_voltage)

        sweep_electrodes = (options.action.__class__ == SweepElectrodesAction)
        self.builder.get_object("textentry_channels")\
            .set_sensitive(options.feedback_enabled and sweep_electrodes)

    def on_radiobutton_retry_toggled(self, widget, data=None):
        """
        Handler called when the "Retry until capacitance..." radio button is
        toggled. 
        """
        logging.debug('retry was toggled %s'\
            % (('OFF', 'ON')[widget.get_active()]))
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        retry = widget.get_active()
        if retry and options.action.__class__ != RetryAction:
            options.action = RetryAction()
        if retry:
            emit_signal('on_step_options_changed',
                        [self.plugin.name, app.protocol.current_step_number],
                        interface=IPlugin)
        
    def on_radiobutton_sweep_frequency_toggled(self, widget, data=None):
        """
        Handler called when the "Sweep Frequency..." radio button is
        toggled. 
        """
        logging.debug('sweep_frequency was toggled %s'\
            % (('OFF', 'ON')[widget.get_active()]))
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        sweep_frequency = widget.get_active()
        if sweep_frequency and options.action.__class__ != SweepFrequencyAction:
            options.action = SweepFrequencyAction()
        if sweep_frequency:
            emit_signal('on_step_options_changed',
                        [self.plugin.name, app.protocol.current_step_number],
                        interface=IPlugin)
        
    def on_radiobutton_sweep_voltage_toggled(self, widget, data=None):
        """
        Handler called when the "Sweep Voltage..." radio button is
        toggled. 
        """
        logging.debug('sweep_voltage was toggled %s'\
            % (('OFF', 'ON')[widget.get_active()]))
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        sweep_voltage = widget.get_active() 
        if sweep_voltage and options.action.__class__!=SweepVoltageAction:
            options.action = SweepVoltageAction()
        if sweep_voltage:
            emit_signal('on_step_options_changed',
                        [self.plugin.name, app.protocol.current_step_number],
                        interface=IPlugin)

    def on_radiobutton_sweep_electrodes_toggled(self, widget, data=None):
        """
        Handler called when the "Sweep Electrodes..." radio button is
        toggled. 
        """
        logging.debug('sweep_electrodes was toggled %s'\
            % (('OFF', 'ON')[widget.get_active()]))
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        sweep_electrodes = widget.get_active() 
        if sweep_electrodes and options.action.__class__!= \
        SweepElectrodesAction:
            options.action = SweepElectrodesAction()
        if sweep_electrodes:
            emit_signal('on_step_options_changed',
                        [self.plugin.name, app.protocol.current_step_number],
                        interface=IPlugin)
        
    def on_textentry_percent_threshold_focus_out_event(self,
                                                       widget,
                                                       event):
        """
        Handler called when the "percent threshold" text box loses focus. 
        """
        self.on_textentry_percent_threshold_changed(widget)
        return False
    
    def on_textentry_percent_threshold_key_press_event(self,
                                                       widget,
                                                       event):
        """
        Handler called when the user presses a key within the "percent
        threshold" text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_percent_threshold_changed(widget)
    
    def on_textentry_percent_threshold_changed(self, widget):
        """
        Update the percent threshold value for the current step. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        options.action.percent_threshold = textentry_validate(widget,
                            options.action.percent_threshold, float)
        emit_signal('on_step_options_changed',
                    [self.plugin.name, app.protocol.current_step_number],
                    interface=IPlugin)

    def on_textentry_increase_voltage_focus_out_event(self, widget, event):
        """
        Handler called when the "increase voltage" text box loses focus. 
        """
        self.on_textentry_increase_voltage_changed(widget)
        return False
    
    def on_textentry_increase_voltage_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "increase
        voltage" text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_increase_voltage_changed(widget)
    
    def on_textentry_increase_voltage_changed(self, widget):
        """
        Update the increase voltage value for the current step. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        options.action.increase_voltage = textentry_validate(widget,
                            options.action.increase_voltage, float)
        emit_signal('on_step_options_changed',
                    [self.plugin.name, app.protocol.current_step_number],
                    interface=IPlugin)
    
    def on_textentry_max_repeats_focus_out_event(self, widget, event):
        """
        Handler called when the "max repeats" text box loses focus. 
        """
        self.on_textentry_max_repeats_changed(widget)
        return False
    
    def on_textentry_max_repeats_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "max repeats"
        text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_max_repeats_changed(widget)
    
    def on_textentry_max_repeats_changed(self, widget):
        """
        Update the max repeats value for the current step. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        options.action.max_repeats = textentry_validate(widget,
                            options.action.max_repeats, int)
        emit_signal('on_step_options_changed',
                    [self.plugin.name, app.protocol.current_step_number],
                    interface=IPlugin)
            
    def on_textentry_start_frequency_focus_out_event(self, widget, event):
        """
        Handler called when the "start frequency" text box loses focus. 
        """
        self.on_textentry_start_frequency_changed(widget)
        return False
    
    def on_textentry_start_frequency_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "start frequency"
        text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_start_frequency_changed(widget)
    
    def on_textentry_start_frequency_changed(self, widget):
        """
        Update the start frequency value for the current step. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        options.action.start_frequency = textentry_validate(widget,
                            options.action.start_frequency / 1e3, float) * 1e3
        emit_signal('on_step_options_changed',
                    [self.plugin.name, app.protocol.current_step_number],
                    interface=IPlugin)

    def on_textentry_end_frequency_focus_out_event(self, widget, event):
        """
        Handler called when the "end frequency" text box loses focus. 
        """
        self.on_textentry_end_frequency_changed(widget)
        return False
    
    def on_textentry_end_frequency_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "end frequency"
        text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_end_frequency_changed(widget)
    
    def on_textentry_end_frequency_changed(self, widget):
        """
        Update the end frequency value for the current step. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        options.action.end_frequency = textentry_validate(widget,
                            options.action.end_frequency / 1e3, float) * 1e3
        emit_signal('on_step_options_changed',
                    [self.plugin.name, app.protocol.current_step_number],
                    interface=IPlugin)

    def on_textentry_n_frequency_steps_focus_out_event(self, widget, event):
        """
        Handler called when the "number of frequency steps" text box loses focus. 
        """
        self.on_textentry_n_frequency_steps_changed(widget)
        return False
    
    def on_textentry_n_frequency_steps_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "number of
        frequency steps" text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_n_frequency_steps_changed(widget)
    
    def on_textentry_n_frequency_steps_changed(self, widget):
        """
        Update the number of frequency steps value for the current step. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        options.action.n_frequency_steps = textentry_validate(widget,
                            options.action.n_frequency_steps, float)
        emit_signal('on_step_options_changed',
                    [self.plugin.name, app.protocol.current_step_number],
                    interface=IPlugin)

    def on_textentry_start_voltage_focus_out_event(self, widget, event):
        """
        Handler called when the "start voltage" text box loses focus. 
        """
        self.on_textentry_start_voltage_changed(widget)
        return False
    
    def on_textentry_start_voltage_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "start voltage"
        text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_start_voltage_changed(widget)
    
    def on_textentry_start_voltage_changed(self, widget):
        """
        Update the start voltage value for the current step. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        options.action.start_voltage = textentry_validate(widget,
                            options.action.start_voltage, float)
        emit_signal('on_step_options_changed',
                    [self.plugin.name, app.protocol.current_step_number],
                    interface=IPlugin)

    def on_textentry_end_voltage_focus_out_event(self, widget, event):
        """
        Handler called when the "end voltage" text box loses focus. 
        """
        self.on_textentry_end_voltage_changed(widget)
        return False
    
    def on_textentry_end_voltage_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "end voltage"
        text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_end_voltage_changed(widget)
    
    def on_textentry_end_voltage_changed(self, widget):
        """
        Update the end voltage value for the current step. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        options.action.end_voltage = textentry_validate(widget,
                            options.action.end_voltage, float)
        emit_signal('on_step_options_changed',
                    [self.plugin.name, app.protocol.current_step_number],
                    interface=IPlugin)

    def on_textentry_n_voltage_steps_focus_out_event(self, widget, event):
        """
        Handler called when the "number of voltage steps" text box loses focus. 
        """
        self.on_textentry_n_voltage_steps_changed(widget)
        return False
    
    def on_textentry_n_voltage_steps_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "number of
        voltage steps" text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_n_voltage_steps_changed(widget)
    
    def on_textentry_n_voltage_steps_changed(self, widget):
        """
        Update the number of voltage steps value for the current step. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        options.action.n_voltage_steps = textentry_validate(widget,
                            options.action.n_voltage_steps, float)
        emit_signal('on_step_options_changed',
                    [self.plugin.name, app.protocol.current_step_number],
                    interface=IPlugin)

    def on_textentry_channels_focus_out_event(self, widget, event):
        """
        Handler called when the "electrodes" text box loses focus. 
        """
        self.on_textentry_channels_changed(widget)
        return False
    
    def on_textentry_channels_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "electrodes"
        text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_channels_changed(widget)
    
    def on_textentry_channels_changed(self, widget):
        """
        Update the electrodes value for the current step. 
        """
        app = get_app()
        all_options = self.plugin.get_step_options()
        options = all_options.feedback_options
        try:
            channels = SetOfInts(widget.get_text())
            assert(min(channels)>=0)
            options.action.channels = channels
            emit_signal('on_step_options_changed',
                        [self.plugin.name, app.protocol.current_step_number],
                        interface=IPlugin)
        except:
            widget.set_text(str(options.action.channels))


class FeedbackResults():
    """
    This class stores the impedance results for a single step in the protocol.
    """
    class_version = str(Version(0,2))
    
    def __init__(self,
                 options,
                 sampling_time_ms,
                 delay_between_samples_ms,                 
                 V_hv,
                 hv_resistor,
                 V_fb,
                 fb_resistor,
                 area,
                 calibration):
        self.options = options
        self.area = area
        self.frequency = options.frequency
        self.V_hv = V_hv
        self.hv_resistor = hv_resistor
        self.V_fb = V_fb
        self.fb_resistor = fb_resistor
        self.time = np.arange(0, math.ceil(options.duration/
                    (sampling_time_ms+delay_between_samples_ms))+2)* \
                    (sampling_time_ms+delay_between_samples_ms)
        self.calibration = calibration
        self.version = self.class_version

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
        logging.debug('[FeedbackResults] version=%s, class_version=%s' % \
                     (str(version), self.class_version))
        if version > Version.fromstring(self.class_version):
            logging.debug('[FeedbackResults] version>class_version')
            raise FutureVersionError(Version.fromstring(self.class_version),
                                     version)
        elif version < Version.fromstring(self.class_version):
            if version < Version(0,1):
                self.calibration = FeedbackCalibration()
            if version < Version(0,2):
                # flag invalid data points
                self.version = str(Version(0,2))
                self.fb_resistor[self.V_fb>5]=-1
                self.hv_resistor[self.V_hv>5]=-1
                logging.info('[FeedbackResults] upgrade to version %s' % \
                            self.version)
        # else the versions are equal and don't need to be upgraded

    def __setstate__(self, state):
        self.__dict__ = state
        for k, v in self.__dict__.items():
            if isinstance(v, list):
                self.__dict__[k] = np.array(v)
        self._upgrade()
        
    def __getstate__(self):
        for k, v in self.__dict__.items():
            if isinstance(v, np.ndarray):
                self.__dict__[k] = v.tolist()
        return self.__dict__

    def V_total(self):
        ind = mlab.find(self.hv_resistor!=-1)
        T = np.zeros(np.array(self.hv_resistor).shape)
        T[ind] = feedback_signal([self.calibration.C_hv[self.hv_resistor[ind]],
                             self.calibration.R_hv[self.hv_resistor[ind]]],
                            self.frequency, 10e6)
        return self.V_hv/T

    def V_actuation(self):
        return self.V_total()-np.array(self.V_fb)

    def Z_device(self):
        ind = mlab.find(self.fb_resistor!=-1)
        R_fb = np.zeros(self.fb_resistor.shape)
        C_fb = np.zeros(self.fb_resistor.shape)
        R_fb[ind] = self.calibration.R_fb[self.fb_resistor[ind]]
        C_fb[ind] = self.calibration.C_fb[self.fb_resistor[ind]]
        return R_fb/np.sqrt(1+np.square(R_fb*C_fb*self.frequency*2*math.pi))* \
            (self.V_total()/self.V_fb - 1)
        
    def min_impedance(self):
        return min(self.Z_device())
    
    def capacitance(self):
        return 1.0/(2*math.pi*self.frequency*self.Z_device())
    
    def dxdt(self, ind=None):
        """
        # remove outliers
        ind = np.nonzero(abs(Z-smooth(Z))/Z>10)[0]
        for j in range(len(ind)-1, -1, -1):
            Z = np.concatenate([Z[:ind[j]],Z[ind[j]+1:]])
            t = np.concatenate([t[:ind[j]],t[ind[j]+1:]])
        """
        if ind is None:
            ind = range(len(self.time))
        window_len = 9
        dt = np.diff(self.time[ind])/1000.0
        Z = self.Z_device()[ind]
        C = self.capacitance()[ind]
        dZdt = self.smooth(np.diff(Z), window_len) / dt
        return -dZdt / Z[:-1]**2/(2*np.pi*self.frequency* \
               C[-1]/np.sqrt(self.area))

    def smooth(self, x, window_len=11, window='hanning'):
        """smooth the data using a window with requested size.
        
        This method is based on the convolution of a scaled window with the signal.
        
        :param x: the input signal 
        :param window_len: the dimension of the smoothing window; should be an odd integer
        :param window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'.  'flat' will produce a moving average smoothing.
    
        :returns: the smoothed signal
            
        **Usage**::
    
        >>t=linspace(-2,2,0.1)
        >>x=sin(t)+randn(len(t))*0.1
        >>y=smooth(x)
        
        .. seealso:: numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve, scipy.signal.lfilter
     
        .. todo:: the window parameter could be the window itself if an array instead of a string   
        """
    
        if x.ndim != 1:
            raise ValueError, "smooth only accepts 1 dimension arrays."
    
        if x.size < window_len:
            raise ValueError, "Input vector needs to be bigger than window size."
    
        if window_len<3:
            return x
    
        if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
            raise ValueError, "Window is none of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'"
    
        s=np.r_[x[0]*np.ones(window_len),x,x[-1]*np.ones(window_len)]
        if window == 'flat': #moving average
            w=np.ones(window_len,'d')
        else:
            w=eval('np.'+window+'(window_len)')
    
        y=np.convolve(w/w.sum(),s,mode='valid')
        y = y[(window_len-1)/2+1:-(window_len-1)/2-1]
        return y


class SweepFrequencyResults():
    """
    This class stores the results for a frequency sweep.
    """
    class_version = str(Version(0,1))
    
    def __init__(self, options, area, calibration):
        self.options = options
        self.area = area
        self.calibration = calibration
        self.frequency = []
        self.V_hv = []
        self.hv_resistor = []
        self.V_fb = []
        self.fb_resistor = []
        self.version = self.class_version

    def _upgrade(self):
        """
        Upgrade the serialized object if necessary.

        Raises:
            FutureVersionError: file was written by a future version of the
                software.
        """
        logging.debug('[SweepFrequencyResults]._upgrade()')
        if hasattr(self, 'version'):
            version = Version.fromstring(self.version)
        else:
            version = Version(0)
        logging.debug('[SweepFrequencyResults] version=%s, class_version=%s' % \
                     (str(version), self.class_version))
        if version > Version.fromstring(self.class_version):
            logging.debug('[SweepFrequencyResults] version>class_version')
            raise FutureVersionError(Version.fromstring(self.class_version),
                                     version)
        elif version < Version.fromstring(self.class_version):
            if version < Version(0,1):
                self.calibration = FeedbackCalibration()
                self.version = str(Version(0,1))
                logging.info('[SweepFrequencyResults] upgrade to version %s' % \
                            self.version)
        # else the versions are equal and don't need to be upgraded

    def __setstate__(self, state):
        self.__dict__ = state
        for k, v in self.__dict__.items():
            if isinstance(v, list) and len(v) and isinstance(v[0], list):
                for i in range(len(v)):
                    if isinstance(self.__dict__[k][i], list):
                        self.__dict__[k][i] = np.array(self.__dict__[k][i])
            elif isinstance(v, list):
                self.__dict__[k] = np.array(v)
        self._upgrade()

    def __getstate__(self):
        for k, v in self.__dict__.items():
            if isinstance(v, list):
                for i in range(len(v)):
                    if isinstance(self.__dict__[k][i], np.ndarray):
                        self.__dict__[k][i] = self.__dict__[k][i].tolist()
                    elif isinstance(self.__dict__[k][i], np.float64):
                        self.__dict__[k][i] = float(self.__dict__[k][i])
            elif isinstance(v, np.ndarray):
                self.__dict__[k] = v.tolist()
        return self.__dict__

    def add_frequency_step(self, frequency,
                           V_hv, hv_resistor,
                           V_fb, fb_resistor):
        self.frequency.append(frequency)
        self.V_hv.append(V_hv)
        self.hv_resistor.append(hv_resistor)
        self.V_fb.append(V_fb)
        self.fb_resistor.append(fb_resistor)

    def V_total(self):
        V = []
        for i in range(0, np.size(self.V_hv, 0)):
            hv_resistor = self.hv_resistor[i]        
            ind = mlab.find(hv_resistor!=-1)
            R_hv = np.zeros(hv_resistor.shape)
            C_hv = np.zeros(hv_resistor.shape)
            R_hv[ind] = self.calibration.R_hv[hv_resistor[ind]]
            C_hv[ind] = self.calibration.C_hv[hv_resistor[ind]]
            T = feedback_signal([C_hv, R_hv], self.frequency[i], 10e6)
            V.append(np.array(self.V_hv[i])/T)
        return V

    def V_actuation(self):
        return self.V_total()-np.array(self.V_fb)

    def Z_device(self):
        Z = []
        V_total = self.V_total()
        for i in range(0, np.size(self.V_hv, 0)):
            fb_resistor = self.fb_resistor[i]            
            ind = mlab.find(fb_resistor!=-1)
            R_fb = np.zeros(fb_resistor.shape)
            C_fb = np.zeros(fb_resistor.shape)
            R_fb[ind] = self.calibration.R_fb[fb_resistor[ind]]
            C_fb[ind] = self.calibration.C_fb[fb_resistor[ind]]
            Z.append(R_fb/np.sqrt(1+np.square(R_fb*C_fb* \
                     self.frequency[i]*2*math.pi))*(V_total[i]/self.V_fb[i]-1))
        return Z
    
    def capacitance(self):
        frequency = np.reshape(np.array(self.frequency),
                               (len(self.frequency),1))
        frequency = np.repeat(frequency, np.size(self.Z_device(), 1), axis=1)
        return 1.0/(2*math.pi*frequency*self.Z_device())


class SweepVoltageResults():
    """
    This class stores the results for a frequency sweep.
    """
    class_version = str(Version(0,1))
    
    def __init__(self, options, area, frequency, calibration):
        self.options = options
        self.area = area
        self.frequency = frequency
        self.calibration = calibration
        self.voltage = []
        self.V_hv = []
        self.hv_resistor = []
        self.V_fb = []
        self.fb_resistor = []
        self.version = self.class_version

    def _upgrade(self):
        """
        Upgrade the serialized object if necessary.

        Raises:
            FutureVersionError: file was written by a future version of the
                software.
        """
        logging.debug('[SweepVoltageResults]._upgrade()')
        if hasattr(self, 'version'):
            version = Version.fromstring(self.version)
        else:
            version = Version(0)
        logging.debug('[SweepVoltageResults] version=%s, class_version=%s' % \
                     (str(version), self.class_version))
        if version > Version.fromstring(self.class_version):
            logging.debug('[SweepVoltageResults] version>class_version')
            raise FutureVersionError(Version.fromstring(self.class_version),
                                     version)
        elif version < Version.fromstring(self.class_version):
            if version < Version(0,1):
                self.calibration = FeedbackCalibration()
                self.version = str(Version(0,1))
                logging.info('[SweepVoltageResults] upgrade to version %s' % \
                            self.version)
        # else the versions are equal and don't need to be upgraded

    def __setstate__(self, state):
        self.__dict__ = state
        for k, v in self.__dict__.items():
            if isinstance(v, list) and len(v) and isinstance(v[0], list):
                for i in range(len(v)):
                    if isinstance(self.__dict__[k][i], list):
                        self.__dict__[k][i] = np.array(self.__dict__[k][i])
            elif isinstance(v, list):
                self.__dict__[k] = np.array(v)
        self._upgrade()

    def __getstate__(self):
        for k, v in self.__dict__.items():
            if isinstance(v, list):
                for i in range(len(v)):
                    if isinstance(self.__dict__[k][i], np.ndarray):
                        self.__dict__[k][i] = self.__dict__[k][i].tolist()
                    elif isinstance(self.__dict__[k][i], np.float64):
                        self.__dict__[k][i] = float(self.__dict__[k][i])
            elif isinstance(v, np.ndarray):
                self.__dict__[k] = v.tolist()
        return self.__dict__

    def add_voltage_step(self, voltage,
                         V_hv, hv_resistor,
                         V_fb, fb_resistor):
        self.voltage.append(voltage)
        self.V_hv.append(V_hv)
        self.hv_resistor.append(hv_resistor)
        self.V_fb.append(V_fb)
        self.fb_resistor.append(fb_resistor)        

    def V_total(self):
        V = []
        for i in range(0, np.size(self.V_hv, 0)):
            hv_resistor = self.hv_resistor[i]        
            ind = mlab.find(hv_resistor!=-1)
            R_hv = np.zeros(hv_resistor.shape)
            C_hv = np.zeros(hv_resistor.shape)
            R_hv[ind] = self.calibration.R_hv[hv_resistor[ind]]
            C_hv[ind] = self.calibration.C_hv[hv_resistor[ind]]
            T = feedback_signal([C_hv, R_hv], self.frequency, 10e6)
            V.append(np.array(self.V_hv[i])/T)
        return V

    def V_actuation(self):
        return self.V_total()-np.array(self.V_fb)

    def Z_device(self):
        Z = []
        V_total = self.V_total()
        for i in range(0, np.size(self.V_hv, 0)):
            fb_resistor = self.fb_resistor[i]            
            ind = mlab.find(fb_resistor!=-1)
            R_fb = np.zeros(fb_resistor.shape)
            C_fb = np.zeros(fb_resistor.shape)
            R_fb[ind] = self.calibration.R_fb[fb_resistor[ind]]
            C_fb[ind] = self.calibration.C_fb[fb_resistor[ind]]
            Z.append(R_fb/np.sqrt(1+np.square(R_fb*C_fb* \
                     self.frequency*2*math.pi))*(V_total[i]/self.V_fb[i]-1))
        return Z

    def capacitance(self):
        return 1.0/(2*math.pi*self.frequency*np.array(self.Z_device()))


class FeedbackResultsController():
    def __init__(self, plugin):
        self.plugin = plugin
        self.builder = gtk.Builder()
        app = get_app()
        self.builder.add_from_file(
            path(app.config['plugins']['directory']).joinpath('dmf_control_board',
                                'microdrop', 'glade', 'feedback_results.glade'))
        self.window = self.builder.get_object("window")
        self.combobox_x_axis = self.builder.get_object("combobox_x_axis")
        self.combobox_y_axis = self.builder.get_object("combobox_y_axis")
        self.window.set_title("Feedback Results")
        self.builder.connect_signals(self)
        self.data = []

        menu_item = gtk.MenuItem("Feedback Results")
        app.main_window_controller.menu_view.append(menu_item)
        menu_item.connect("activate", self.on_window_show)
        menu_item.show()

        self.figure = Figure()
        self.canvas = FigureCanvasGTK(self.figure)
        self.axis = self.figure.add_subplot(111)
        self.vbox = self.builder.get_object("vbox1")
        toolbar = NavigationToolbar(self.canvas, self.window)
        self.vbox.pack_start(self.canvas)
        self.vbox.pack_start(toolbar, False, False)
        combobox_set_model_from_list(self.combobox_x_axis,
                                     ["Time", "Frequency", "Voltage"])
        combobox_set_model_from_list(self.combobox_y_axis,
                                     ["Impedance", "Capacitance", "Velocity",
                                      "Voltage"])
        self.combobox_x_axis.set_active(0)
        self.combobox_y_axis.set_active(0)

    def on_window_show(self, widget, data=None):
        """
        Handler called when the user clicks on "Feedback Results" in the "View"
        menu.
        """
        self.window.show_all()

    def on_window_delete_event(self, widget, data=None):
        """
        Handler called when the user closes the "Feedback Results" window. 
        """
        self.window.hide()
        return True

    def on_combobox_x_axis_changed(self, widget, data=None):
        x_axis = combobox_get_active_text(self.combobox_x_axis)
        y_axis = combobox_get_active_text(self.combobox_y_axis)
        if x_axis=="Time":
            combobox_set_model_from_list(self.combobox_y_axis,
                                         ["Impedance", "Capacitance",
                                          "Velocity", "Voltage"])
        else:
            combobox_set_model_from_list(self.combobox_y_axis,
                                         ["Impedance", "Capacitance",
                                          "Voltage"])
        self.combobox_y_axis.set_active(0)
        self.update_plot()

    def on_combobox_y_axis_changed(self, widget, data=None):
        self.update_plot()

    def on_export_data_clicked(self, widget, data=None):
        dialog = gtk.FileChooserDialog(title="Export data",
                                       action=gtk.FILE_CHOOSER_ACTION_SAVE,
                                       buttons=(gtk.STOCK_CANCEL,
                                                gtk.RESPONSE_CANCEL,
                                                gtk.STOCK_SAVE,
                                                gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        dialog.set_current_name("export.csv")
        filter = gtk.FileFilter()
        filter.set_name("*.csv")
        filter.add_pattern("*.csv")
        dialog.add_filter(filter)
        filter = gtk.FileFilter()
        filter.set_name("All files")
        filter.add_pattern("*")
        dialog.add_filter(filter)
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            filename = dialog.get_filename()
            logging.info("Exporting to file %s." % filename)
            try:
                with open(filename, 'w') as f:
                    f.write("\n".join(self.export_data))
            except Exception, e:
                logging.error("Problem exporting file. %s." % e)
        dialog.destroy()

    def on_experiment_log_selection_changed(self, data):
        """
        Handler called whenever the experiment log selection changes.

        :param data: experiment log data (list of dictionaries, one per step) for the selected steps
        """
        self.data = data
        self.update_plot()

    def update_plot(self):
        x_axis = combobox_get_active_text(self.combobox_x_axis)
        y_axis = combobox_get_active_text(self.combobox_y_axis)
        self.axis.cla()
        self.axis.grid(True)
        legend = []
        legend_loc = "upper right"
        self.export_data = []        
        if x_axis=="Time":
            self.axis.set_xlabel("Time (ms)")
            for row in self.data:
                if self.plugin.name in row.keys() and "FeedbackResults" in row[self.plugin.name].keys():
                    results = row[self.plugin.name]["FeedbackResults"]
                    self.export_data.append('step:, %d' % (row['core']["step"]+1))
                    self.export_data.append('step time (s):, %f' % (row['core']["time"]))

                    # only plot values that have a valid fb and hv resistor
                    ind = mlab.find(np.logical_and(results.fb_resistor!=-1,
                                    results.hv_resistor!=-1))
                    
                    if y_axis=="Impedance":
                        self.axis.set_title("Impedance")
                        self.axis.set_ylabel(
                            "|Z$_{device}$(f=%.1e Hz)| ($\Omega$)" % \
                            results.frequency)
                        self.axis.plot(results.time[ind],
                                       results.Z_device()[ind])
                        self.axis.set_yscale('log')
                        self.export_data.append('time (ms):, '+
                            ", ".join([str(x) for x in results.time]))
                        self.export_data.append('impedance (Ohms):, ' + 
                            ", ".join([str(x) for x in results.Z_device()]))
                    elif y_axis=="Capacitance":
                        self.axis.set_title("Capacitance/Area")
                        self.axis.set_ylabel("C$_{device}$ (F/mm$^2$)")
                        self.axis.plot(results.time[ind],
                                       results.capacitance()[ind]/results.area)
                        legend_loc = "lower right"
                        self.export_data.append('time (ms):, '+
                            ", ".join([str(x) for x in results.time]))
                        self.export_data.append('capacitance/area (F/mm^2):,' + 
                            ", ".join([str(x) for x in results.capacitance()]))
                    elif y_axis=="Velocity":
                        dxdt = results.dxdt(ind)
                        t = (np.array(results.time[:-1]) + \
                             np.array(results.time[1:]))[ind[:-1]]/2
                        self.axis.set_title("Instantaneous velocity")
                        self.axis.set_ylabel("Velocity$_{drop}$ (mm/s)")
                        self.axis.plot(t, dxdt)
                        self.export_data.append('time (ms):, '+
                            ", ".join([str(x) for x in (
                                np.array(results.time[:-1])+
                                np.array(results.time[1:]))/2]))
                        self.export_data.append('velocity (mm/s):,' + 
                            ", ".join([str(x) for x in dxdt]))
                    elif y_axis=="Voltage":
                        self.axis.set_title("Actuation voltage")
                        self.axis.set_ylabel("V$_{actuation}$ (V$_{RMS}$)")
                        self.axis.plot(results.time[ind],
                                       results.V_actuation()[ind])
                        legend_loc = "lower right"
                        self.export_data.append('time (ms):, '+
                            ", ".join([str(x) for x in results.time]))
                        self.export_data.append('V_actuation (V_RMS):,' + 
                            ", ".join([str(x) for x in results.V_actuation()]))
                    legend.append("Step %d (%.3f s)" % (row['core']["step"]+1,
                                                        row['core']["time"]))
        elif x_axis=="Frequency":
            self.axis.set_xlabel("Frequency (Hz)")
            for row in self.data:
                if self.plugin.name in row.keys() and \
                "SweepFrequencyResults" in row[self.plugin.name].keys():
                    results = row[self.plugin.name]["SweepFrequencyResults"]
                    self.export_data.append('step:, %d' % \
                                            (row['core']["step"]+1))
                    self.export_data.append('step time (s):, %f' % \
                                            (row['core']["time"]))
                    self.export_data.append('frequency (Hz):, '+
                        ", ".join([str(x) for x in results.frequency]))
                    if y_axis=="Impedance":
                        self.axis.set_title("Impedance")
                        self.axis.set_ylabel("|Z$_{device}$(f)| ($\Omega$)")
                        self.axis.errorbar(results.frequency,
                                           np.mean(results.Z_device(), 1),
                                           np.std(results.Z_device(), 1),
                                           fmt='.')
                        self.axis.set_xscale('log')
                        self.axis.set_yscale('log')
                        self.export_data.append('mean(impedance) (Ohms):, ' + 
                            ", ".join([str(x) for x in np.mean(
                            results.Z_device(), 1)]))
                        self.export_data.append('std(impedance) (Ohms):, ' + 
                            ", ".join([str(x) for x in np.std(
                            results.Z_device(), 1)]))
                    elif y_axis=="Capacitance":
                        self.axis.set_title("Capacitance/Area")
                        self.axis.set_ylabel("C$_{device}$ (F/mm$^2$)")
                        self.axis.errorbar(results.frequency,
                                           np.mean(results.capacitance(), 1)/
                                                results.area,
                                           np.std(results.capacitance(), 1)/
                                                results.area,
                                           fmt='.')
                        self.axis.set_xscale('log')
                        self.export_data.append('mean(capacitance/area) '
                            '(F/mm^2):, ' + ", ".join([str(x) for x in np.mean(
                            results.Z_device(), 1)]))
                        self.export_data.append('std(capacitance/area) '
                            '(F/mm^2):, ' + ", ".join([str(x) for x in np.std(
                            results.Z_device(), 1)]))
                    elif y_axis=="Voltage":
                        self.axis.set_title("Actuation voltage")
                        self.axis.set_ylabel("V$_{actuation}$ (V$_{RMS}$)")
                        self.axis.errorbar(results.frequency,
                                           np.mean(results.V_actuation(), 1),
                                           np.std(results.V_actuation(), 1),
                                           fmt='.')
                        self.axis.set_xscale('log')
                        legend_loc = "lower right"
                        self.export_data.append('mean(V_actuation) '
                            '(Vrms):, ' + ", ".join([str(x) for x in np.mean(
                            results.V_actuation(), 1)]))
                        self.export_data.append('std(V_actuation) '
                            '(Vrms):, ' + ", ".join([str(x) for x in np.std(
                            results.V_actuation(), 1)]))
                    legend.append("Step %d (%.3f s)" % \
                                  (row['core']["step"]+1, row['core']["time"]))
        elif x_axis=="Voltage":
            self.axis.set_xlabel("Actuation Voltage (V$_{RMS}$)")
            for row in self.data:
                if self.plugin.name in row.keys() and \
                "SweepVoltageResults" in row[self.plugin.name].keys():
                    results = row[self.plugin.name]["SweepVoltageResults"]
                    self.export_data.append('step:, %d' % \
                                            (row['core']["step"]+1))
                    self.export_data.append('step time (s):, %f' % \
                                            (row['core']["time"]))
                    self.export_data.append('voltage (Vrms):, '+
                        ", ".join([str(x) for x in results.voltage]))
                    if y_axis=="Impedance":
                        self.axis.set_title("Impedance")
                        self.axis.set_ylabel(
                            "|Z$_{device}$(f=%.1e Hz)| ($\Omega$)" % \
                            results.frequency)
                        self.axis.errorbar(results.voltage,
                                           np.mean(results.Z_device(), 1),
                                           np.std(results.Z_device(), 1),
                                           fmt='.')
                        self.axis.set_yscale('log')
                        self.export_data.append('mean(impedance) (Ohms):, ' + 
                            ", ".join([str(x) for x in np.mean(
                            results.Z_device(), 1)]))
                        self.export_data.append('std(impedance) (Ohms):, ' + 
                            ", ".join([str(x) for x in np.std(
                            results.Z_device(), 1)]))
                    elif y_axis=="Capacitance":
                        self.axis.set_title("Capacitance/Area")
                        self.axis.set_ylabel("C$_{device}$ (F/mm$^2$)")
                        self.axis.errorbar(results.voltage,
                                           np.mean(results.capacitance()/results.area, 1),
                                           np.std(results.capacitance()/results.area, 1),
                                           fmt='.')
                        self.export_data.append('mean(capacitance/area) '
                            '(F/mm^2):, ' + ", ".join([str(x) for x in np.mean(
                            results.Z_device(), 1)]))
                        self.export_data.append('std(capacitance/area) '
                            '(F/mm^2):, ' + ", ".join([str(x) for x in np.std(
                            results.Z_device(), 1)]))
                    elif y_axis=="Voltage":
                        self.axis.set_title("Actuation voltage")
                        self.axis.set_ylabel("V$_{actuation}$ (V$_{RMS}$)")
                        self.axis.errorbar(results.voltage,
                                           np.mean(results.V_actuation(), 1),
                                           np.std(results.V_actuation(), 1),
                                           fmt='.')
                        legend_loc = "lower right"
                        self.export_data.append('mean(V_actuation) '
                            '(Vrms):, ' + ", ".join([str(x) for x in np.mean(
                            results.V_actuation(), 1)]))
                        self.export_data.append('std(V_actuation) '
                            '(Vrms):, ' + ", ".join([str(x) for x in np.std(
                            results.V_actuation(), 1)]))
                    legend.append("Step %d (%.3f s)" % (row['core']["step"]+1,
                                                        row['core']["time"]))
        if len(legend):
            self.axis.legend(legend, loc=legend_loc)
        self.figure.subplots_adjust(left=0.17, bottom=0.15)
        self.canvas.draw()


class FeedbackCalibrationController():
    def __init__(self, plugin):
        self.plugin = plugin
        
    def on_perform_calibration(self, widget, data=None):
        if not self.plugin.control_board.connected():
            logging.error("A control board must be connected in order to "
                          "perform calibration.")
            return
            
        response = yesno("Would you like to calibrate the high voltage "
                         "attenuators? Click no to keep current values")
        if response == gtk.RESPONSE_YES:
            self.calibrate_attenuators()

        response = yesno("Would you like to calibrate the feedback resistors? "
                         "Please note that you must have an electrode covered "
                         "by a drop and that electrode should be actuated. If "
                         "the device is not ready, press \"No\", setup the drop "
                         "and relaunch the calibration wizard.")
        
        if response == gtk.RESPONSE_YES:
            self.calibrate_feedback_resistors()

    def calibrate_attenuators(self):
        frequencies = self.prompt_for_frequency_range()
        
        app = get_app()
        app.main_window_controller.info("The control board uses a bank of "
            "resistors to measure amplifier voltage. These resistors need to "
            "be characterized in order to obtain accurate measurements. Using "
            "an oscilloscope, please measure the output from your amplifier "
            "and answer the following prompts.",
            "HV attenuator calibration wizard")

        results = self.sweep_frequencies(frequencies,
                                         input_voltage=None,
                                         measure_with_scope=True,
                                         autoscale_voltage=True)

        if results:
            dialog = gtk.FileChooserDialog(title="Save feedback calibration",
                                           action=gtk.FILE_CHOOSER_ACTION_SAVE,
                                           buttons=(gtk.STOCK_CANCEL,
                                                    gtk.RESPONSE_CANCEL,
                                                    gtk.STOCK_SAVE,
                                                    gtk.RESPONSE_OK))
            dialog.set_default_response(gtk.RESPONSE_OK)
            response = dialog.run()
            if response == gtk.RESPONSE_OK:
                filename = path(dialog.get_filename())
                with open(filename.abspath(), 'wb') as f:
                    pickle.dump(results, f)
            dialog.destroy()
            self.process_hv_calibration(results)

    def prompt_for_frequency_range(self, title="Perform calibration"):
        form = Form.of(
            Integer.named('start_frequency').using(default=1e2, optional=True,
                validators=[ValueAtLeast(minimum=99), ]),
            Integer.named('end_frequency').using(default=30e3, optional=True,
                validators=[ValueAtLeast(minimum=1e3), ]),
            Integer.named('number_of_steps').using(default=10, optional=True,
                validators=[ValueAtLeast(minimum=0), ]),
        )
        dialog = FormViewDialog()
        valid, response =  dialog.run(form)
        start_frequency = np.array(response['start_frequency'])
        end_frequency = np.array(response['end_frequency'])
        number_of_steps = np.array(response['number_of_steps'])
        frequencies = np.logspace(np.log10(start_frequency),
                                  np.log10(end_frequency),
                                  number_of_steps)
        return frequencies

    def calibrate_feedback_resistors(self):
        app = get_app()
        state = app.dmf_device_controller.get_step_options().state_of_channels
        options = self.plugin.get_default_step_options()
        if len(mlab.find(state>0))==0:
            logging.error("Can't calibrate feedback resistors because no "
                "electrodes are on.")
            return
        else:
            max_channels = self.plugin.control_board.number_of_channels() 
            if len(state) >  max_channels:
                state = state[0:max_channels]
            elif len(state) < max_channels:
                state = np.concatenate([state,
                        np.zeros(max_channels - len(state), int)])
            else:
                assert(len(state) == max_channels)
        
        frequencies = self.prompt_for_frequency_range()
        n_samples = 1000
        calibration = self.plugin.control_board.calibration
        V_hv = np.zeros([len(frequencies),
                                len(calibration.R_fb)])
        V_fb = np.zeros([len(frequencies),
                         len(calibration.R_fb)])
        app_values = self.plugin.get_app_values()

        for i, frequency in enumerate(frequencies):
            options.frequency = frequency
            emit_signal("set_frequency", frequency,
                interface=IWaveformGenerator)
            for j in range(len(calibration.R_fb)):
                self.plugin.control_board.set_series_resistor_index(1,j)
                options.voltage=100
                while True:
                    emit_signal("set_voltage", options.voltage,
                                interface=IWaveformGenerator)
                    (v_hv, hv_resistor, v_fb, fb_resistor) = \
                        self.plugin.check_impedance(options)

                    # wait for the signal to settle
                    time.sleep(.2)
                    
                    results = FeedbackResults(options,
                        app_values['sampling_time_ms'],
                        app_values['delay_between_samples_ms'],
                        v_hv,
                        hv_resistor,
                        v_fb,
                        fb_resistor,
                        self.plugin.get_actuated_area(),
                        self.plugin.control_board.calibration)
                    V_hv[i, j] = results.V_total()[-1]
                    gain = self.plugin.control_board.amplifier_gain()
                    self.plugin.control_board.set_state_of_all_channels(state)
                    
                    # wait for the signal to settle
                    time.sleep(.2)
                    
                    V = np.array(self.plugin.control_board.analog_reads(1,
                        n_samples))/1024.0*5-2.5
                    V_rms = (np.max(V)-np.min(V))/np.sqrt(2)/2
                    logging.info("V_rms=%.1f" % V_rms)
                    if V_rms > 1.4:
                        logging.info("divide input voltage by 2")
                        options.voltage /= 2
                    elif V_rms < .3 and options.voltage/gain < np.sqrt(2)/2:
                        logging.info("double voltage")
                        options.voltage *= 2
                    else:
                        V_fb[i, j] = V_rms
                        break

        results = dict(V_hv=V_hv.tolist(),
                       frequencies = frequencies.tolist(), 
                       V_fb=V_fb.tolist())

        if results:
            dialog = gtk.FileChooserDialog(title="Save feedback calibration",
                                           action=gtk.FILE_CHOOSER_ACTION_SAVE,
                                           buttons=(gtk.STOCK_CANCEL,
                                                    gtk.RESPONSE_CANCEL,
                                                    gtk.STOCK_SAVE,
                                                    gtk.RESPONSE_OK))
            dialog.set_default_response(gtk.RESPONSE_OK)
            response = dialog.run()
            if response == gtk.RESPONSE_OK:
                filename = path(dialog.get_filename())
                with open(filename.abspath(), 'wb') as f:
                    pickle.dump(results, f)
            dialog.destroy()
            self.process_fb_calibration(results)

    def sweep_frequencies(self,
                          frequencies,
                          input_voltage=None,
                          measure_with_scope=False,
                          autoscale_voltage=True):
        n_samples = 1000
        n_attenuation_steps = len(self.plugin.control_board.calibration.R_hv)

        if input_voltage is None:
            input_voltage = .5*np.ones([n_attenuation_steps,
                                        len(frequencies)]
            )

        if measure_with_scope:
            voltages = np.zeros([n_attenuation_steps,
                                len(frequencies)])

        hv_measurements = np.zeros([n_attenuation_steps,
                                    len(frequencies),                                    
                                    n_samples])
        
        self.plugin.control_board.set_amplifier_gain(1.0)        

        for i in range(0, n_attenuation_steps):
            self.plugin.control_board.set_series_resistor_index(0,i)
            for j, frequency in enumerate(frequencies):
                emit_signal("set_voltage", input_voltage[i, j],
                            interface=IWaveformGenerator)
                emit_signal("set_frequency", frequency,
                    interface=IWaveformGenerator)

                # adjust reference voltage so that we are reading ~1.4 Vrms
                if autoscale_voltage:
                    while True:
                        logging.info('set_voltage(%.5f)' % input_voltage[i, j])
                        emit_signal("set_voltage", input_voltage[i, j],
                                    interface=IWaveformGenerator)
                        # wait for the signal to settle
                        time.sleep(.1)
                        V = np.array(self.plugin.control_board.analog_reads(0,
                            n_samples))/1024.0*5-2.5
                        V_rms = (np.max(V)-np.min(V))/np.sqrt(2)/2
                        logging.info("V_rms=%.1f" % V_rms)
                        if V_rms > 1.3:
                            logging.info("divide input voltage by 2")
                            input_voltage[i, j] /= 2
                        # maximum of waveform generator is ~4Vpp = sqrt(2) Vrms
                        elif V_rms <.5 and input_voltage[i, j] < np.sqrt(2)/2:
                            logging.info("multiply input voltage by 2")
                            input_voltage[i, j] *= 2
                        else:
                            logging.info("input voltage set to %.1f" % \
                                         input_voltage[i, j])
                            break
                
                if measure_with_scope:
                    while True:
                        voltage = text_entry_dialog("What is the current RMS output voltage?",
                                                    title="Feedback calibration wizard")
                        # cancel calibration
                        if voltage is None:
                            get_app().main_window_controller.info("Calibration cancelled.",
                                "Feedback calibration wizard")
                            return
                        # check that it is a valid voltage
                        elif is_float(voltage):
                            voltages[i, j] = float(voltage)
                            break
                        # try again
                        else:
                            logging.error("Not a valid float.")

                hv_measurements[i,j,:] = self.plugin.control_board. \
                    analog_reads(0, n_samples)


        results = dict(input_voltage=input_voltage.tolist(),
                       frequencies = frequencies.tolist(), 
                       hv_measurements=hv_measurements.tolist())

        if measure_with_scope:
            results['voltages']=voltages.tolist()
        return results
            
    def on_load_calibration_from_file(self, widget, data=None):
        dialog = gtk.FileChooserDialog(
            title="Load attenuator calibration from file",
            action=gtk.FILE_CHOOSER_ACTION_OPEN,
            buttons=(gtk.STOCK_CANCEL,
                     gtk.RESPONSE_CANCEL,
                     gtk.STOCK_OPEN,
                     gtk.RESPONSE_OK)
        )
        dialog.set_default_response(gtk.RESPONSE_OK)
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            filename = path(dialog.get_filename())
            results = None
            with open(filename, 'rb') as f:
                try:
                    results = pickle.load(f)
                    logging.debug("Loaded object from pickle.")
                except Exception, e:
                    logging.debug("Not a valid pickle file. %s." % e)
            if results==None:
                with open(filename, 'rb') as f:
                    try:
                        results = yaml.load(f)
                        logging.debug("Loaded object from YAML file.")
                    except Exception, e:
                        logging.debug("Not a valid YAML file. %s." % e)
            if not results:
                raise TypeError
            if 'V_fb' in results and 'V_hv' in results and \
                'frequencies' in results:
                self.process_fb_calibration(results)
            else:
                self.process_hv_calibration(results)
        dialog.destroy()

    def process_fb_calibration(self, results):
        calibration = self.plugin.control_board.calibration
        frequencies = np.array(results['frequencies'])
        V_hv = np.array(results['V_hv'])
        V_fb = np.array(results['V_fb'])

        voltage_filter = [.5, 1.3]
        # only include data points where the voltage falls within a specified
        # range
        x,y = np.nonzero(np.logical_or(V_fb<voltage_filter[0],
                                       V_fb>voltage_filter[1]))
        V_fb[x, y] = 0

        schema_entries = []
        for i in range(len(self.plugin.control_board.calibration.R_hv)):
            schema_entries.append(
                Boolean.named('Fit R_hv_%d' % i).using(
                    default=True, optional=True))
            schema_entries.append(
                Boolean.named('Fit C_hv_%d' % i).using(
                    default=True, optional=True))
        for i in range(len(self.plugin.control_board.calibration.R_fb)):
            schema_entries.append(
                Boolean.named('Fit R_fb_%d' % i).using(
                    default=True, optional=True))
            schema_entries.append(
                Boolean.named('Fit C_fb_%d' % i).using(
                    default=True, optional=True))
        schema_entries.append(
            Float.named('C_device_pF').using(default=1, optional=True))
        
        form = Form.of(*schema_entries)
        dialog = FormViewDialog('Fit paramters')
        valid, response =  dialog.run(form)

        # fit parameters
        C_device = response['C_device_pF']*1e-12
        p0 = np.array([calibration.R_fb[0],
                       calibration.R_fb[1],
                       calibration.R_fb[2],
                       calibration.R_fb[3],
                       calibration.C_fb[0],
                       calibration.C_fb[1],
                       calibration.C_fb[2],
                       calibration.C_fb[3],
        ])

        def e(p0, V_hv, V_fb, frequencies, C):
            R_fb = np.concatenate((p0[0]*np.ones([V_fb.shape[0], 1]),
                                   p0[1]*np.ones([V_fb.shape[0], 1]),
                                   p0[2]*np.ones([V_fb.shape[0], 1]),
                                   p0[3]*np.ones([V_fb.shape[0], 1])), 1)
            C_fb = np.concatenate((p0[4]*np.ones([V_fb.shape[0], 1]),
                                   p0[5]*np.ones([V_fb.shape[0], 1]),
                                   p0[6]*np.ones([V_fb.shape[0], 1]),
                                   p0[7]*np.ones([V_fb.shape[0], 1])), 1)
            f = np.tile(np.reshape(frequencies, (len(frequencies), 1)),
                        (1, V_fb.shape[1]))
            error = V_fb-V_hv*R_fb*C*2*np.pi*f/np.sqrt(1+np.square(2*np.pi*R_fb*(C_fb+C)*f))
            return error.flatten()[mlab.find(np.logical_and(V_hv.flatten(), V_fb.flatten()))]

        p1, cov_x, infodict, mesg, ier = optimize.leastsq(
            e,
            p0,
            args=(V_hv, V_fb, frequencies, C_device),
            full_output=True
        )
        
        print "p0=", p0
        print "SOS before=", np.sum(e(p0, V_hv, V_fb, frequencies, C_device)**2)

        print "p1=", p1, mesg, ier
        print "SOS after=", np.sum(e(p1, V_hv, V_fb, frequencies, C_device)**2)
        print "diff=", p1-p0

        Z_0 = self.device_impedance(p0, V_hv, V_fb, frequencies)

        canvas, a = self.create_plot('V_hv')
        legend = []
        for i in range(Z_0.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.semilogx(frequencies[ind], V_hv[ind, i], 'o')
        a.legend(legend)
        canvas.draw()

        canvas, a = self.create_plot('V_fb')
        legend = []
        for i in range(Z_0.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.semilogx(frequencies[ind], V_fb[ind, i], 'o')
        a.legend(legend)
        canvas.draw()

        canvas, a = self.create_plot('Device impedance')
        legend = []
        for i in range(Z_0.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.loglog(frequencies[ind], Z_0[ind, i], 'o')
        a.plot(frequencies, 1/(2*np.pi*C_device*frequencies), 'k--')
        a.legend(legend)
        canvas.draw()

        canvas, a = self.create_plot('Device capacitance')
        legend = []
        for i in range(Z_0.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.semilogx(frequencies[ind], 1/(Z_0[ind, i]*frequencies[ind]*2*np.pi), 'o')
        a.plot(frequencies, C_device*np.ones(frequencies.shape), 'k--')
        a.legend(legend)
        canvas.draw()

        Z_1 = self.device_impedance(p1, V_hv, V_fb, frequencies)

        canvas, a = self.create_plot('Device impedance (after fit)')
        legend = []
        for i in range(Z_1.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.loglog(frequencies[ind], Z_1[ind, i], 'o')
        a.plot(frequencies, 1/(2*np.pi*C_device*frequencies), 'k--')
        a.legend(legend)
        canvas.draw()
        
        canvas, a = self.create_plot('Device capacitance (after fit)')
        legend = []
        for i in range(Z_1.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.semilogx(frequencies[ind], 1/(Z_1[ind, i]*frequencies[ind]*2*np.pi), 'o')
        a.plot(frequencies, C_device*np.ones(frequencies.shape), 'k--')
        a.legend(legend)
        canvas.draw()

        canvas, a = self.create_plot('V_fb/V_hv')
        legend = []
        colors = ['b','r','c','m']
        for i in range(Z_1.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.loglog(frequencies[ind], V_fb[ind, i]/V_hv[ind, i], colors[i]+'o')
        for i in range(Z_1.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                R_fb = p1[0+i]
                C_fb = p1[4+i]
                a.plot(frequencies[ind], R_fb*C_device*2*np.pi*frequencies[ind]/ \
                         np.sqrt(1+np.square(2*np.pi*R_fb*(C_fb+C_device)*frequencies[ind])), colors[i]+'--')
        a.legend(legend)
        canvas.draw()

        # write new calibration parameters to the control board
        for i in range(0, len(calibration.R_fb)):
            self.plugin.control_board.set_series_resistor_index(1,i)
            self.plugin.control_board.set_series_resistance(1, abs(p1[i]))
            self.plugin.control_board.set_series_capacitance(1, abs(p1[4+i]))
        # reconnect to update settings
        self.plugin.control_board.connect()
        
    def device_impedance(self, p0, V_hv, V_fb, frequencies):
        R_fb = np.concatenate((p0[0]*np.ones([V_fb.shape[0], 1]),
                               p0[1]*np.ones([V_fb.shape[0], 1]),
                               p0[2]*np.ones([V_fb.shape[0], 1]),
                               p0[3]*np.ones([V_fb.shape[0], 1])), 1)
        C_fb = np.concatenate((p0[4]*np.ones([V_fb.shape[0], 1]),
                               p0[5]*np.ones([V_fb.shape[0], 1]),
                               p0[6]*np.ones([V_fb.shape[0], 1]),
                               p0[7]*np.ones([V_fb.shape[0], 1])), 1)
        f = np.tile(np.reshape(frequencies, (len(frequencies), 1)),
                    (1, V_fb.shape[1]))

        return R_fb/np.sqrt(1+np.square(2*np.pi*R_fb*C_fb*f))*(V_hv/V_fb-1)

    def process_hv_calibration(self, results):
        input_voltage = np.array(results['input_voltage'])
        frequencies = np.array(results['frequencies'])
        hv_measurements = np.array(results['hv_measurements'])/1024.0*5-2.5
        hv_rms = np.transpose(np.array([np.max(hv_measurements[:, j, :],1) - \
                           np.min(hv_measurements[:, j, :],1) \
                           for j in range(0, len(frequencies))])/2./np.sqrt(2))

        if 'voltages' in results:
            voltages = np.array(results['voltages'])
            attenuation = hv_rms/voltages
        else:
            attenuation = hv_rms/input_voltage

        # p[0]=C, p[1]=R2
        R1=10e6
        f = lambda p, x, R1: np.abs(p[1]/(R1+p[1]+R1*p[1]*2*np.pi*p[0]*complex(0,1)*x))
        e = lambda p, x, y, R1: f(p, x, R1) - y
        fit_params = []
    
        canvas, a = self.create_plot('HV attenuation')
        colors = ['b', 'r']
        legend = []
        for i in range(0, np.size(hv_rms,0)):
            ind = mlab.find(hv_rms[i,:]>.1)
            p0 = [self.plugin.control_board.calibration.C_hv[i],
                  self.plugin.control_board.calibration.R_hv[i]
            ]
            a.loglog(frequencies[ind], f(p0, frequencies[ind], R1), colors[i]+'--')
            legend.append('Ch%d, fit' % i)

            if 'voltages' in results:
                voltages = np.array(results['voltages'])
                T=attenuation[i,ind]
                p1, success = optimize.leastsq(e, p0, args=(frequencies[ind], T, R1))
                fit_params.append(p1)
                a.loglog(frequencies[ind], f(p1, frequencies[ind], R1), colors[i]+'-')
                legend.append('Ch%d, new fit' % i)

                a.plot(frequencies, attenuation[i], colors[i]+'o')
                legend.append('Ch%d, scope' % i)

                # update control board calibration
                self.plugin.control_board.set_series_resistor_index(0,i)
                self.plugin.control_board.set_series_resistance(0, p1[1])
                self.plugin.control_board.set_series_capacitance(0, p1[0])
                # reconnnect to update calibration data
                self.plugin.control_board.connect()
            else:
                fit_params.append(p0)
                a.plot(frequencies, attenuation[i], colors[i]+'o')
                legend.append('Ch%d, CB measurements' % i)
                
        a.legend(legend)
        a.set_xlabel('Frequency (Hz)')
        a.set_ylabel('Attenuation')
        a.set_title('HV attenuation')
        canvas.draw()

        i=0
        legend = []
        fit_params = np.array(fit_params)
        canvas, a = self.create_plot("Relative amplifier gain")
        ind = mlab.find(hv_rms[i,:]>.1)
        gain = None
        if 'voltages' in results:
            gain = voltages[i, :]/input_voltage[i, :]
            a.semilogx(frequencies, gain, 'bo')
            legend.append('Ch%d, scope' % i)
        
        #a.semilogx(frequencies, gain/gain[0], 'bo')
        #legend.append('No feedback')
        a.semilogx(frequencies,
                   hv_rms[i,:]/f(fit_params[i,:], frequencies, R1)/input_voltage[i, :],
                   'ro')
        legend.append('Feedback')
        a.set_xlabel('Frequency (Hz)')
        a.set_ylabel('Relative gain')
        a.set_title("V$_o$/V$_{in}$")
        a.legend(legend, loc="upper left")
        canvas.draw()

    def create_plot(self, title):
        win = gtk.Window()
        win.set_default_size(500, 400)
        win.set_title(title)
        f = Figure()
        a = f.add_subplot(111)
        canvas = FigureCanvasGTK(f)
        vbox = gtk.VBox(False, 0)
        win.add(vbox)
        toolbar = NavigationToolbar(canvas, win)
        vbox.pack_start(canvas)
        vbox.pack_start(toolbar, False, False)
        vbox.show()
        win.show_all()
        f.subplots_adjust(left=0.12, bottom=0.16)
        return (canvas, a)
