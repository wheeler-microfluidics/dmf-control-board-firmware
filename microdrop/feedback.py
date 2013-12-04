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
from plugin_manager import emit_signal, IWaveformGenerator, IPlugin, \
    get_service_instance_by_name
from app_context import get_app


class AmplifierGainNotCalibrated(Exception):
    pass


def feedback_signal(p, frequency, Z, hw_version):
    """p[0]=C, p[1]=R"""
    if hw_version.major==1:
        return np.abs(1/(Z/p[1]+1+Z*2*np.pi*p[0]*complex(0,1)*frequency))
    else:
        return np.abs(1/(Z/p[1]+Z*2*np.pi*p[0]*complex(0,1)*frequency))

class RetryAction():
    class_version = str(Version(0,1))

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
            self.feedback_options_menu_item = gtk.MenuItem("Feedback Options")
            self.plugin.control_board_menu.append(self.feedback_options_menu_item)
            self.feedback_options_menu_item.connect("activate", self.on_window_show)
            self.feedback_options_menu_item.show()
            self.feedback_options_menu_item.set_sensitive(
                app.dmf_device is not None)
            
            self.measure_cap_filler_menu_item = gtk.MenuItem(
                    "Measure capacitance of filler media")
            app.dmf_device_controller.view.popup.add_item(
                    self.measure_cap_filler_menu_item)
            self.measure_cap_filler_menu_item.connect("activate",
                    self.on_measure_cap_filler)
            self.measure_cap_liquid_menu_item = gtk.MenuItem(
                    "Measure capacitance of liquid")
            app.dmf_device_controller.view.popup.add_item(
                    self.measure_cap_liquid_menu_item)
            self.measure_cap_liquid_menu_item.connect("activate",
                    self.on_measure_cap_liquid)

            self.initialized = True
        self.measure_cap_filler_menu_item.show()
        self.measure_cap_liquid_menu_item.show()

    def on_plugin_disable(self):
        self.measure_cap_filler_menu_item.hide()
        self.measure_cap_liquid_menu_item.hide()

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

    def on_measure_cap_filler(self, widget, data=None):
        self.plugin.control_board.calibration.C_filler = \
            self.measure_device_capacitance()

    def on_measure_cap_liquid(self, widget, data=None):
        self.plugin.control_board.calibration.C_drop = \
            self.measure_device_capacitance()

    def measure_device_capacitance(self):
        app = get_app()
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
            self.plugin.control_board.measure_impedance(
                app_values['sampling_time_ms'],
                int(math.ceil(test_options.duration/ 
                    app_values['sampling_time_ms'])),
                app_values['delay_between_samples_ms'],
                state)
        results = FeedbackResults(test_options,
            app_values['sampling_time_ms'],
            app_values['delay_between_samples_ms'],
            V_hv, hv_resistor,
            V_fb, fb_resistor,
            area,
            self.plugin.control_board.calibration,
            0)
        logging.info('max(results.capacitance())/area=%s' % (max(results.capacitance()) / area))
        self.plugin.control_board.state_of_all_channels = current_state
        return max(results.capacitance()) / area

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
    class_version = str(Version(0,3))
    
    def __init__(self,
                 options,
                 sampling_time_ms,
                 delay_between_samples_ms,                 
                 V_hv,
                 hv_resistor,
                 V_fb,
                 fb_resistor,
                 area,
                 calibration,
                 attempt):
        self.options = options
        self.area = area
        self.frequency = options.frequency
        self.V_hv = V_hv
        self.hv_resistor = hv_resistor
        self.V_fb = V_fb
        self.fb_resistor = fb_resistor
        self.time = np.arange(0, math.ceil(options.duration/
                    (sampling_time_ms+delay_between_samples_ms)))* \
                    (sampling_time_ms+delay_between_samples_ms)
        self.calibration = calibration
        attempt = attempt
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
            if version < Version(0,3):
                self.attempt=0
                self.version = str(Version(0,3))
                logging.info('[FeedbackResults] upgrade to version %s' % \
                            self.version)
        # else the versions are equal and don't need to be upgraded

    def __setstate__(self, state):
        # convert lists to numpy arrays
        self.__dict__ = state
        for k, v in self.__dict__.items():
            if isinstance(v, list):
                self.__dict__[k] = np.array(v)
        self._upgrade()
        
    def __getstate__(self):
        # convert numpy arrays/floats to standard lists/floats
        out = deepcopy(self.__dict__)
        for k, v in out.items():
            if isinstance(v, np.ndarray):
                out[k] = v.tolist()
        return out

    def V_total(self):
        ind = mlab.find(self.hv_resistor!=-1)
        T = np.zeros(self.hv_resistor.shape)
        T[ind] = feedback_signal([self.calibration.C_hv[self.hv_resistor[ind]],
                             self.calibration.R_hv[self.hv_resistor[ind]]],
                            self.frequency, 10e6, self.calibration.hw_version)
        return self.V_hv/T

    def V_actuation(self):
        if self.calibration.hw_version.major == 1:
            return self.V_total()-np.array(self.V_fb)
        else:
            return self.V_total()

    def Z_device(self):
        ind = mlab.find(self.fb_resistor!=-1)
        R_fb = np.zeros(self.fb_resistor.shape)
        C_fb = np.zeros(self.fb_resistor.shape)
        R_fb[ind] = self.calibration.R_fb[self.fb_resistor[ind]]
        C_fb[ind] = self.calibration.C_fb[self.fb_resistor[ind]]
        if self.calibration.hw_version.major == 1:
            return R_fb/np.sqrt(1+np.square(R_fb*C_fb*self.frequency*2*math.pi))* \
                (self.V_total()/self.V_fb - 1)
        else:
            return R_fb/np.sqrt(1+np.square(R_fb*C_fb*self.frequency*2*math.pi))* \
                (self.V_total()/self.V_fb)
        
    def min_impedance(self):
        return min(self.Z_device())
    
    def capacitance(self):
        return 1.0/(2*math.pi*self.frequency*self.Z_device())
    
    def x_position(self):
        if self.calibration.C_drop:
            C_drop = self.calibration.C_drop
        else:
            C_drop = self.capacitance()[-1]/self.area
        if self.calibration.C_filler:
            C_filler = self.calibration.C_filler
        else:
            C_filler = 0
        return (self.capacitance()/self.area-C_filler)/ \
            (C_drop-C_filler)*np.sqrt(self.area)

    def mean_velocity(self, ind=None, threshold=0.95):
        if ind is None:
            ind = range(len(self.time))
        t, dxdt = self.dxdt(ind, threshold)
        x = self.x_position()
        ind_stop = ind[mlab.find(dxdt==0)[0]]
        dx = x[ind_stop]-x[ind[0]]
        if max(dxdt>0) and ind_stop<len(t):
            dt = t[ind_stop]-t[ind[0]]
            return dx/dt
        else:
            return 0

    def dxdt(self, ind=None, threshold=0.95):
        if ind is None:
            ind = range(len(self.time))
        dt = np.diff(self.time[ind])
        t = self.time[ind][1:]-(self.time[1]-self.time[0])/2.0
        C = self.capacitance()[ind]
        dCdt = np.diff(C)/dt
        
        if self.calibration.C_drop:
            C_drop = self.calibration.C_drop*self.area
        else:
            C_drop = C[-1]
        if self.calibration.C_filler:
            C_filler = self.calibration.C_filler*self.area
        else:
            C_filler = 0
        
        # find the time when the capacitance exceeds the specified threshold
        # (e.g., the drop has stopped moving once it has passed 95% of it's
        # final value)
        ind_stop = mlab.find((C[1:]-C_filler)/(C[-1]-C_filler)>threshold)
        if len(ind_stop):
            # set all remaining velocities to 0
            dCdt[ind_stop[0]:]=0

        return t, dCdt/(C_drop-C_filler)*np.sqrt(self.area)


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
        # convert lists to numpy arrays
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
        # convert numpy arrays/floats to standard lists/floats
        out = deepcopy(self.__dict__)
        for k, v in out.items():
            if isinstance(v, list):
                for i in range(len(v)):
                    if isinstance(out[k][i], np.ndarray):
                        out[k][i] = out[k][i].tolist()
                    elif isinstance(out[k][i], np.float64):
                        out[k][i] = float(out[k][i])
            elif isinstance(v, np.ndarray):
                out[k] = v.tolist()
        return out

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
            hv_resistor = np.array(self.hv_resistor[i])
            ind = mlab.find(hv_resistor!=-1)
            R_hv = np.zeros(hv_resistor.shape)
            C_hv = np.zeros(hv_resistor.shape)
            R_hv[ind] = self.calibration.R_hv[hv_resistor[ind]]
            C_hv[ind] = self.calibration.C_hv[hv_resistor[ind]]
            T = feedback_signal([C_hv, R_hv], self.frequency[i],
                                10e6, self.calibration.hw_version)
            V.append(np.array(self.V_hv[i])/T)
        return V

    def V_actuation(self):
        if self.calibration.hw_version.major == 1:
            return self.V_total()-np.array(self.V_fb)
        else:
            return self.V_total()

    def Z_device(self):
        Z = []
        V_total = self.V_total()
        for i in range(0, np.size(self.V_hv, 0)):
            fb_resistor = np.array(self.fb_resistor[i])
            ind = mlab.find(fb_resistor!=-1)
            R_fb = np.zeros(fb_resistor.shape)
            C_fb = np.zeros(fb_resistor.shape)
            R_fb[ind] = self.calibration.R_fb[fb_resistor[ind]]
            C_fb[ind] = self.calibration.C_fb[fb_resistor[ind]]
            if self.calibration.hw_version.major == 1:
                Z.append(R_fb/np.sqrt(1+np.square(R_fb*C_fb* \
                         self.frequency[i]*2*math.pi))* \
                         (V_total[i]/self.V_fb[i]-1))
            else:
                Z.append(R_fb/np.sqrt(1+np.square(R_fb*C_fb* \
                         self.frequency[i]*2*math.pi))* \
                         (V_total[i]/self.V_fb[i]))
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
        # convert lists to numpy arrays
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
        # convert numpy arrays/floats to standard lists/floats
        out = deepcopy(self.__dict__)
        for k, v in out.items():
            if isinstance(v, list):
                for i in range(len(v)):
                    if isinstance(out[k][i], np.ndarray):
                        out[k][i] = out[k][i].tolist()
                    elif isinstance(out[k][i], np.float64):
                        out[k][i] = float(out[k][i])
            elif isinstance(v, np.ndarray):
                out[k] = v.tolist()
        return out

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
            hv_resistor = np.array(self.hv_resistor[i])
            ind = mlab.find(hv_resistor!=-1)
            R_hv = np.zeros(hv_resistor.shape)
            C_hv = np.zeros(hv_resistor.shape)
            R_hv[ind] = self.calibration.R_hv[hv_resistor[ind]]
            C_hv[ind] = self.calibration.C_hv[hv_resistor[ind]]
            T = feedback_signal([C_hv, R_hv], self.frequency,
                                10e6, self.calibration.hw_version)
            V.append(np.array(self.V_hv[i])/T)
        return V

    def V_actuation(self):
        if self.calibration.hw_version.major == 1:
            return self.V_total()-np.array(self.V_fb)
        else:
            return self.V_total()

    def Z_device(self):
        Z = []
        V_total = self.V_total()
        for i in range(0, np.size(self.V_hv, 0)):
            fb_resistor = np.array(self.fb_resistor[i])            
            ind = mlab.find(fb_resistor!=-1)
            R_fb = np.zeros(fb_resistor.shape)
            C_fb = np.zeros(fb_resistor.shape)
            R_fb[ind] = self.calibration.R_fb[fb_resistor[ind]]
            C_fb[ind] = self.calibration.C_fb[fb_resistor[ind]]
            if self.calibration.hw_version.major == 1:
                Z.append(R_fb/np.sqrt(1+np.square(R_fb*C_fb* \
                         self.frequency*2*math.pi))* \
                         (V_total[i]/self.V_fb[i]-1))
            else:
                Z.append(R_fb/np.sqrt(1+np.square(R_fb*C_fb* \
                         self.frequency*2*math.pi))* \
                         (V_total[i]/self.V_fb[i]))
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
        self.checkbutton_normalize_by_area = self.builder.get_object(
            "checkbutton_normalize_by_area")
        self.window.set_title("Feedback Results")
        self.builder.connect_signals(self)
        self.data = []

        self.feedback_results_menu_item = gtk.MenuItem("Feedback Results")
        app.main_window_controller.menu_view.append(
            self.feedback_results_menu_item)
        self.feedback_results_menu_item.connect("activate", self.on_window_show)

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
                                      "Voltage", "x-position"])
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
                                          "Velocity",  "Voltage", "x-position"])
        else:
            combobox_set_model_from_list(self.combobox_y_axis,
                                         ["Impedance", "Capacitance",
                                          "Voltage"])
        self.combobox_y_axis.set_active(0)
        self.update_plot()

    def on_combobox_y_axis_changed(self, widget, data=None):
        y_axis = combobox_get_active_text(self.combobox_y_axis)
        self.checkbutton_normalize_by_area.set_sensitive(y_axis=="Impedance" \
            or y_axis=="Capacitance")
        self.update_plot()

    def on_checkbutton_normalize_by_area_toggled(self, widget, data=None):
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

        normalization_string = ""
        if self.checkbutton_normalize_by_area.get_active():
            normalization_string = "/mm$^2$"
        
        if y_axis=="Impedance":
            self.axis.set_title("Impedance%s" % \
                                normalization_string)
            self.axis.set_ylabel(
                "|Z$_{device}$| ($\Omega$%s)" % normalization_string)
            self.axis.set_yscale('log')
        elif y_axis=="Capacitance":
            self.axis.set_title("Capacitance%s" % \
                                normalization_string)
            self.axis.set_ylabel("C$_{device}$ (F%s)" % \
                                 normalization_string)
            legend_loc = "lower right"
        elif y_axis=="Velocity":
            self.axis.set_title("Instantaneous velocity")
            self.axis.set_ylabel("Velocity$_{drop}$ (mm/s)")
        elif y_axis=="Voltage":
            self.axis.set_title("Actuation voltage")
            self.axis.set_ylabel("V$_{actuation}$ (V$_{RMS}$)")
            legend_loc = "lower right"
        elif y_axis=="x-position":
            self.axis.set_title("x-position")
            self.axis.set_ylabel("x-position (mm)")

        if x_axis=="Time":
            self.axis.set_xlabel("Time (ms)")
            for row in self.data:
                if self.plugin.name in row.keys() and "FeedbackResults" in row[self.plugin.name].keys():
                    results = row[self.plugin.name]["FeedbackResults"]

                    normalization = 1.0
                    if self.checkbutton_normalize_by_area.get_active():
                        normalization = results.area

                    self.export_data.append('step:, %d' % (row['core']["step"]+1))
                    self.export_data.append('step time (s):, %f' % (row['core']["time"]))

                    # only plot values that have a valid fb and hv resistor,
                    # and that have been using the same fb and hv resistor for
                    # > 1 consecutive measurement
                    ind = mlab.find(np.logical_and(
                        np.logical_and(
                        results.fb_resistor!=-1,
                        results.hv_resistor!=-1),
                        np.logical_and(
                        np.concatenate(([0],np.diff(results.fb_resistor)))==0,
                        np.concatenate(([0],np.diff(results.hv_resistor)))==0
                    )))
                        
                    if y_axis=="Impedance":
                        self.axis.plot(results.time[ind],
                                       results.Z_device()[ind]/normalization)
                        self.export_data.append('time (ms):, '+
                            ", ".join([str(x) for x in results.time[ind]]))
                        self.export_data.append('impedance (Ohms%s):, ' % \
                                                (normalization_string) + \
                            ", ".join([str(x) for x in \
                                       results.Z_device()[ind]/normalization]))
                    elif y_axis=="Capacitance":
                        self.axis.plot(results.time[ind],
                                       results.capacitance()[ind]/normalization)
                        self.export_data.append('time (ms):, '+
                            ", ".join([str(x) for x in results.time[ind]]))
                        self.export_data.append('capacitance (F%s):,' % \
                                                normalization_string + \
                            ", ".join([str(x) for x in \
                                       results.capacitance()[ind]/ \
                                       normalization]))
                    elif y_axis=="Velocity":
                        t, dxdt = results.dxdt(ind)
                        self.axis.plot(t, dxdt*1000)
                        self.export_data.append('time (ms):, '+
                            ", ".join([str(x) for x in t]))
                        self.export_data.append('velocity (mm/s):,' + 
                            ", ".join([str(x) for x in dxdt]))
                    elif y_axis=="Voltage":
                        self.axis.plot(results.time[ind],
                                       results.V_actuation()[ind])
                        self.export_data.append('time (ms):, '+
                            ", ".join([str(x) for x in results.time[ind]]))
                        self.export_data.append('V_actuation (V_RMS):,' + 
                            ", ".join([str(x) for x in \
                                       results.V_actuation()[ind]]))
                    elif y_axis=="x-position":
                        t = results.time[ind]
                        x_pos = results.x_position()[ind]
                        self.axis.plot(t, x_pos)
                        self.export_data.append('time (ms):, '+
                            ", ".join([str(x) for x in t]))
                        self.export_data.append('velocity (mm/s):,' + 
                            ", ".join([str(x) for x in x_pos]))
                    legend.append("Step %d (%.3f s)" % (row['core']["step"]+1,
                                                        row['core']["time"]))
        elif x_axis=="Frequency":
            self.axis.set_xlabel("Frequency (Hz)")
            self.axis.set_xscale('log')
            for row in self.data:
                if self.plugin.name in row.keys() and \
                "SweepFrequencyResults" in row[self.plugin.name].keys():
                    results = row[self.plugin.name]["SweepFrequencyResults"]
                    
                    normalization = 1.0
                    if self.checkbutton_normalize_by_area.get_active():
                        normalization = results.area
                    
                    self.export_data.append('step:, %d' % \
                                            (row['core']["step"]+1))
                    self.export_data.append('step time (s):, %f' % \
                                            (row['core']["time"]))
                    self.export_data.append('frequency (Hz):, '+
                        ", ".join([str(x) for x in results.frequency]))
                    if y_axis=="Impedance":
                        self.axis.errorbar(results.frequency,
                                           np.mean(results.Z_device(), 1)/ \
                                           normalization,
                                           np.std(results.Z_device(), 1)/ \
                                           normalization,
                                           fmt='.')
                        self.export_data.append('mean(impedance) (Ohms%s):, ' \
                            % normalization_string + \
                            ", ".join([str(x) for x in np.mean(
                            results.Z_device(), 1)/normalization]))
                        self.export_data.append('std(impedance) (Ohms%s):, ' \
                            % normalization_string + \
                            ", ".join([str(x) for x in np.std(
                            results.Z_device(), 1)/normalization]))
                    elif y_axis=="Capacitance":
                        self.axis.errorbar(results.frequency,
                                           np.mean(results.capacitance(), 1)/ \
                                           normalization,
                                           np.std(results.capacitance(), 1)/ \
                                           normalization,
                                           fmt='.')
                        self.export_data.append('mean(capacitance) '
                            '(F):, ' + ", ".join([str(x) for x in np.mean(
                            results.capacitance()/normalization, 1)]))
                        self.export_data.append('std(capacitance/area) '
                            '(F):, ' + ", ".join([str(x) for x in np.std(
                            results.capacitance(), 1)/normalization]))
                    elif y_axis=="Voltage":
                        self.axis.errorbar(results.frequency,
                                           np.mean(results.V_actuation(), 1),
                                           np.std(results.V_actuation(), 1),
                                           fmt='.')
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
                    
                    normalization = 1.0
                    if self.checkbutton_normalize_by_area.get_active():
                        normalization = results.area
                    
                    self.export_data.append('step:, %d' % \
                                            (row['core']["step"]+1))
                    self.export_data.append('step time (s):, %f' % \
                                            (row['core']["time"]))
                    self.export_data.append('voltage (Vrms):, '+
                        ", ".join([str(x) for x in results.voltage]))
                    if y_axis=="Impedance":
                        self.axis.errorbar(results.voltage,
                                           np.mean(results.Z_device(), 1)/ \
                                           normalization,
                                           np.std(results.Z_device(), 1)/ 
                                           normalization,
                                           fmt='.')
                        self.export_data.append('mean(impedance) (Ohms%s):, ' \
                            % normalization_string + \
                            ", ".join([str(x) for x in np.mean(
                            results.Z_device(), 1)/normalization]))
                        self.export_data.append('std(impedance) (Ohms%s):, ' \
                            % normalization_string + \
                            ", ".join([str(x) for x in np.std(
                            results.Z_device(), 1)/normalization]))
                    elif y_axis=="Capacitance":
                        self.axis.errorbar(results.voltage,
                                           np.mean(results.capacitance(), 1)/ \
                                           normalization,
                                           np.std(results.capacitance(), 1)/ \
                                           normalization,
                                           fmt='.')
                        self.export_data.append('mean(capacitance) '
                            '(F):, ' + ", ".join([str(x) for x in np.mean(
                            results.capacitance(), 1)/normalization]))
                        self.export_data.append('std(capacitance) '
                            '(F):, ' + ", ".join([str(x) for x in np.std(
                            results.capacitance(), 1)/normalization]))
                    elif y_axis=="Voltage":
                        self.axis.errorbar(results.voltage,
                                           np.mean(results.V_actuation(), 1),
                                           np.std(results.V_actuation(), 1),
                                           fmt='.')
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
        self.experiment_log_controller = get_service_instance_by_name(
            "microdrop.gui.experiment_log_controller", "microdrop")

    def on_save_log_calibration(self, widget, data=None):
        selected_data = self.experiment_log_controller.get_selected_data()
        calibration = None
        if len(selected_data)>1:
            logger.error("Multiple steps are selected. Please choose a single "
                         "step.")
            return
        try:
            if 'FeedbackResults' in selected_data[0][self.plugin.name]:
                calibration = selected_data[0][self.plugin.name] \
                    ['FeedbackResults'].calibration
            elif 'SweepFrequencyResults' in selected_data[0][self.plugin.name]:
                calibration = selected_data[0][self.plugin.name] \
                    ['SweepFrequencyResults'].calibration
            elif 'SweepVoltageResults' in selected_data[0][self.plugin.name]:
                calibration = selected_data[0][self.plugin.name] \
                    ['SweepVoltageResults'].calibration
        except:
            logger.error("This step does not contain any calibration data.")
            return
        
        dialog = gtk.FileChooserDialog(title="Save feedback calibration",
                                       action=gtk.FILE_CHOOSER_ACTION_SAVE,
                                       buttons=(gtk.STOCK_CANCEL,
                                                gtk.RESPONSE_CANCEL,
                                                gtk.STOCK_SAVE,
                                                gtk.RESPONSE_OK))

        while True:
            try:
                dialog.set_default_response(gtk.RESPONSE_OK)
                response = dialog.run()
                if response == gtk.RESPONSE_OK:
                    filename = path(dialog.get_filename())
                    with open(filename.abspath(), 'wb') as f:
                        pickle.dump(calibration, f)
                    break
                else:
                    break
            except Exception, why:
                logger.error("Error saving calibration file. %s." % why)
        dialog.destroy()
    
    def on_load_log_calibration(self, widget, data=None):
        dialog = gtk.FileChooserDialog(
            title="Load calibration from file",
            action=gtk.FILE_CHOOSER_ACTION_OPEN,
            buttons=(gtk.STOCK_CANCEL,
                     gtk.RESPONSE_CANCEL,
                     gtk.STOCK_OPEN,
                     gtk.RESPONSE_OK)
        )
        dialog.set_default_response(gtk.RESPONSE_OK)
        response = dialog.run()
        calibration = None
        if response == gtk.RESPONSE_OK:
            filename = path(dialog.get_filename())
            with open(filename, 'rb') as f:
                try:
                    calibration = pickle.load(f)
                    logging.debug("Loaded object from pickle.")
                    if str(calibration.__class__).split('.')[-1] != \
                        'FeedbackCalibration':
                        raise ValueError()
                except Exception, why:
                    logging.error('Not a valid calibration file.')
                    logging.debug(why)
        dialog.destroy()

        selected_data = self.experiment_log_controller.get_selected_data()
        for row in selected_data:
            try:
                if 'FeedbackResults' in row[self.plugin.name]:
                    row[self.plugin.name]['FeedbackResults'].calibration = \
                        deepcopy(calibration)
                elif 'SweepFrequencyResults' in row[self.plugin.name]:
                    row[self.plugin.name]['SweepFrequencyResults']. \
                    calibration = deepcopy(calibration)
                elif 'SweepVoltageResults' in row[self.plugin.name]:
                    row[self.plugin.name]['SweepVoltageResults'].calibration = \
                        deepcopy(calibration)
            except:
                continue
        # save the experiment log with the new values
        filename = os.path.join(self.experiment_log_controller.results. \
                                log.directory,
                                str(self.experiment_log_controller.results. \
                                    log.experiment_id),
                                'data')
        self.experiment_log_controller.results.log.save(filename)
        emit_signal("on_experiment_log_selection_changed", [selected_data])   
    
    def on_edit_log_calibration(self, widget, data=None):
        logger.debug("on_edit_log_calibration()")
        settings = {}
        schema_entries = []
        calibration_list = []
        selected_data = self.experiment_log_controller.get_selected_data()
        for row in selected_data:
            try:
                if 'FeedbackResults' in row[self.plugin.name]:
                    calibration_list.append(row[self.plugin.name] \
                                        ['FeedbackResults'].calibration)
                elif 'SweepFrequencyResults' in row[self.plugin.name]:
                    calibration_list.append(row[self.plugin.name] \
                                        ['SweepFrequencyResults'].calibration)
                elif 'SweepVoltageResults' in row[self.plugin.name]:
                    calibration_list.append(row[self.plugin.name] \
                                        ['SweepVoltageResults'].calibration)
            except:
                continue

            calibration = calibration_list[-1]
            
            # set default for each setting only if all selected steps have a
            # the same value, otherwise, leave the default blank
            if len(calibration_list)==1:
                settings["C_drop"] = calibration.C_drop
                settings["C_filler"] = calibration.C_filler
                for i in range(len(calibration.R_hv)):
                    settings['R_hv_%d' % i] = calibration.R_hv[i]
                    settings['C_hv_%d' % i] = calibration.C_hv[i]
                for i in range(len(calibration.R_fb)):
                    settings['R_fb_%d' % i] = calibration.R_fb[i]
                    settings['C_fb_%d' % i] = calibration.C_fb[i]
            else:
                def check_group_value(name, new):
                    if settings[name] and settings[name] != new:
                        settings[name] = None
                check_group_value("C_drop", calibration.C_drop)
                check_group_value("C_filler", calibration.C_filler)
                for i in range(len(calibration.R_hv)):
                    check_group_value('R_hv_%d' % i, calibration.R_hv[i])
                    check_group_value('C_hv_%d' % i, calibration.C_hv[i])
                for i in range(len(calibration.R_fb)):
                    check_group_value('R_fb_%d' % i, calibration.R_fb[i])
                    check_group_value('C_fb_%d' % i, calibration.C_fb[i])

        def set_field_value(name, multiplier=1):
            string_value = ""
            if settings[name]:
                string_value = str(settings[name]*multiplier)
            schema_entries.append(String.named(name).using(
                default=string_value, optional=True))

        set_field_value('C_drop', 1e12)
        set_field_value('C_filler', 1e12)

        for i in range(len(calibration.R_hv)):
            set_field_value('R_hv_%d' % i)
            set_field_value('C_hv_%d' % i, 1e12)
        for i in range(len(calibration.R_fb)):
            set_field_value('R_fb_%d' % i)
            set_field_value('C_fb_%d' % i, 1e12)
        
        form = Form.of(*schema_entries)
        dialog = FormViewDialog('Edit calibration settings')
        valid, response =  dialog.run(form)
        
        if not valid:
            return

        logger.debug("Applying updated calibration settings to log file.")

        def get_field_value(name, multiplier=1):
            try:
                logger.debug('response[%s]=' % name, response[name])
                logger.debug('settings[%s]=' % name, settings[name])
                if response[name] and \
                    (settings[name] is None or abs(float(response[name])/multiplier-settings[name])/ \
                    settings[name] > .0001):
                    return float(response[name])/multiplier
            except ValueError:
                logger.error('C_drop value (%s) is invalid.' % response['C_drop'])
            return None
        
        value = get_field_value('C_drop', 1e12)
        if value:
            for calibration in calibration_list:
                calibration.C_drop = value
        value = get_field_value('C_filler', 1e12)
        if value:
            for calibration in calibration_list:
                calibration.C_filler = value
        for i in range(len(calibration.R_hv)):
            value = get_field_value('R_hv_%d' % i)
            if value:
                for calibration in calibration_list:
                    calibration.R_hv[i] = value
            value = get_field_value('C_hv_%d' % i, 1e12)
            if value:
                for calibration in calibration_list:
                    calibration.C_hv[i] = value
        for i in range(len(calibration.R_fb)):
            value = get_field_value('R_fb_%d' % i)
            if value:
                for calibration in calibration_list:
                    calibration.R_fb[i] = value
            value = get_field_value('C_fb_%d' % i, 1e12)
            if value:
                for calibration in calibration_list:
                    calibration.C_fb[i] = value
                
        # save the experiment log with the new values
        filename = os.path.join(self.experiment_log_controller.results. \
                                log.directory,
                                str(self.experiment_log_controller.results. \
                                    log.experiment_id),
                                'data')
        self.experiment_log_controller.results.log.save(filename)
        emit_signal("on_experiment_log_selection_changed", [selected_data])   

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
        frequencies, valid = self.prompt_for_frequency_range()
        if not valid:
            return
        
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
            while True:
                try:
                    dialog.set_default_response(gtk.RESPONSE_OK)
                    response = dialog.run()
                    if response == gtk.RESPONSE_OK:
                        filename = path(dialog.get_filename())
                        with open(filename.abspath(), 'wb') as f:
                            pickle.dump(results, f)
                        break
                    else:
                        break
                except Exception, why:
                    logger.error("Error saving calibration file. %s." % why)
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
        return frequencies, valid

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
        
        frequencies, valid = self.prompt_for_frequency_range()
        if not valid:
            return
        n_samples = 1000
        calibration = self.plugin.control_board.calibration
        V_hv = np.zeros([len(frequencies),
                                len(calibration.R_fb)])
        V_fb = np.zeros([len(frequencies),
                         len(calibration.R_fb)])
        app_values = self.plugin.get_app_values()

        for i, frequency in enumerate(frequencies):
            options.frequency = frequency
            sampling_time_ms = int(max(1000.0/frequency*5, 10))
            options.duration = 10*sampling_time_ms
            emit_signal("set_frequency", frequency,
                interface=IWaveformGenerator)
            for j in range(0, len(calibration.R_fb)):
                logging.info("set_series_resistor_index(1, %d)" % j)
                self.plugin.control_board.set_series_resistor_index(1,j)
                # start with the minimum voltage
                options.voltage=2.0
                while True:
                    emit_signal("set_voltage", options.voltage,
                                interface=IWaveformGenerator)
                    (v_hv, hv_resistor, v_fb, fb_resistor) = \
                        self.plugin.control_board.measure_impedance(
                            sampling_time_ms,
                            int(math.ceil(options.duration/ 
                                sampling_time_ms)),
                            app_values['delay_between_samples_ms'],
                            state)

                    results = FeedbackResults(options,
                        sampling_time_ms,
                        app_values['delay_between_samples_ms'],
                        v_hv,
                        hv_resistor,
                        v_fb,
                        fb_resistor,
                        self.plugin.get_actuated_area(),
                        self.plugin.control_board.calibration,
                        0)
                    logging.info("gain=%.1f" % self.plugin.control_board.amplifier_gain())
                    V_hv[i, j] = np.mean(results.V_total()[5:])
                    
                    time.sleep(.1)
                    
                    V = self.plugin.control_board.analog_reads(1,
                        n_samples)/1023.0*5-2.5
                    V_rms = (np.max(V)-np.min(V))/2/np.sqrt(2)
                    logging.info("V_rms=%.1f" % V_rms)
                    # TODO check for max voltage (or catch errors when setting voltage)
                    if V_rms < .3 and options.voltage < 400.0/2:
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
            input_voltage = .1*np.ones([n_attenuation_steps,
                                        len(frequencies)]
            )

        if measure_with_scope:
            voltages = np.zeros([n_attenuation_steps,
                                len(frequencies)])

        hv_measurements = np.zeros([n_attenuation_steps,
                                    len(frequencies),                                    
                                    n_samples])
        gain = self.plugin.control_board.amplifier_gain()
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
                            n_samples))/1023.0*5-2.5
                        V_rms = np.sqrt(np.mean(V**2)-np.mean(V)**2)
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

        # reset amplifier gain to previous value
        self.plugin.control_board.set_amplifier_gain(gain)

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
        filename = path(dialog.get_filename())
        dialog.destroy()
        if response == gtk.RESPONSE_OK:
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
                logging.error("Not a valid calibration file")
            if 'V_fb' in results and 'V_hv' in results and \
                'frequencies' in results:
                self.process_fb_calibration(results)
            else:
                self.process_hv_calibration(results)

    def process_fb_calibration(self, results):
        calibration = self.plugin.control_board.calibration
        frequencies = np.array(results['frequencies'])
        V_hv = np.array(results['V_hv'])
        V_fb = np.array(results['V_fb'])

        voltage_filter = [.1, 1.3]
        # only include data points where the voltage falls within a specified
        # range
        x,y = np.nonzero(np.logical_or(V_fb<voltage_filter[0],
                                       V_fb>voltage_filter[1]))
        V_fb[x, y] = 0

        schema_entries = []
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
        p0 = np.concatenate((calibration.R_fb, calibration.C_fb))

        def e(p0, V_hv, V_fb, frequencies, C):
            R_fb = []
            for i in range(0, len(p0)/2):
                R_fb.append((p0[i]*np.ones(V_fb.shape[0])).tolist())
            R_fb = np.abs(np.array(R_fb).transpose())
            C_fb = []
            for i in range(len(p0)/2, len(p0)):
                C_fb.append((p0[i]*np.ones(V_fb.shape[0])).tolist())
            C_fb = np.abs(np.array(C_fb).transpose())
            f = np.tile(np.reshape(frequencies, (len(frequencies), 1)),
                        (1, V_fb.shape[1]))
            if calibration.hw_version.major == 1:
                error = V_fb-V_hv*R_fb*C*2*np.pi*f/np.sqrt(1+np.square(2*np.pi*R_fb*(C_fb+C)*f))
            else:
                error = V_fb-V_hv*R_fb*C*2*np.pi*f/np.sqrt(1+np.square(2*np.pi*R_fb*C_fb*f))
            return error.flatten()[mlab.find(np.logical_and(V_hv.flatten(), V_fb.flatten()))]
        
        p1, cov_x, infodict, mesg, ier = optimize.leastsq(
            e,
            p0,
            args=(V_hv, V_fb, frequencies, C_device),
            full_output=True
        )
        p1 = np.abs(p1)
        
        logger.info("p0=%s" % p0)
        logger.info("SOS before=%s" % np.sum(e(p0, V_hv, V_fb, frequencies, C_device)**2))
        logger.info("p1=%s" % p1)
        logger.info(mesg)
        logger.info("SOS after=%s" % np.sum(e(p1, V_hv, V_fb, frequencies, C_device)**2))
        logger.info("diff=%s" % (p1-p0))
        
        canvas, a = self.create_plot('residuals')
        legend = []
        a.plot(e(p1, V_hv, V_fb, frequencies, C_device), 'o')
        a.set_xlabel('data point')
        a.set_ylabel('residual')
        a.set_title('Residuals from fit')
        canvas.draw()

        Z_0 = self.device_impedance(p0, V_hv, V_fb, frequencies)

        canvas, a = self.create_plot('V_hv')
        legend = []
        for i in range(Z_0.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.semilogx(frequencies[ind], V_hv[ind, i], 'o')
        a.legend(legend)
        a.set_xlabel('Frequency (Hz)')
        a.set_ylabel('V$_{hv}$ (V$_{RMS}$)')
        a.set_title('V$_{hv}$')
        canvas.draw()

        canvas, a = self.create_plot('V_fb')
        legend = []
        for i in range(Z_0.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.semilogx(frequencies[ind], V_fb[ind, i], 'o')
        a.legend(legend)
        a.set_xlabel('Frequency (Hz)')
        a.set_ylabel('V$_{fb}$ (V$_{RMS}$)')
        a.set_title('V$_{fb}$')
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
        a.set_xlabel('Frequency (Hz)')
        a.set_ylabel('Z$_{device}$ ($\Omega$)')
        a.set_title('Z$_{device}$')
        canvas.draw()

        canvas, a = self.create_plot('Device capacitance')
        legend = []
        for i in range(Z_0.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.semilogx(frequencies[ind], 1e12/(Z_0[ind, i]*frequencies[ind]*2*np.pi), 'o')
        a.plot(frequencies, C_device*np.ones(frequencies.shape), 'k--')
        a.legend(legend)
        a.set_xlabel('Frequency (Hz)')
        a.set_ylabel('C$_{device}$ (pF)')
        a.set_title('C$_{device}$')
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
        a.set_xlabel('Frequency (Hz)')
        a.set_ylabel('Z$_{device}$ ($\Omega$)')
        a.set_title('Z$_{device}$ (after fit)')
        canvas.draw()
        
        canvas, a = self.create_plot('Device capacitance (after fit)')
        legend = []
        for i in range(Z_1.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.semilogx(frequencies[ind], 1e12/(Z_1[ind, i]*frequencies[ind]*2*np.pi), 'o')
        a.plot(frequencies, C_device*np.ones(frequencies.shape), 'k--')
        a.legend(legend)
        a.set_xlabel('Frequency (Hz)')
        a.set_ylabel('C$_{device}$ (pF)')
        a.set_title('C$_{device}$ (after fit)')
        canvas.draw()

        canvas, a = self.create_plot('V_fb/V_hv')
        legend = []
        colors = ['b','g','r','c','m']
        color_index = 0
        for i in range(Z_1.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                legend.append("R$_{fb,%d}$" % i)
                a.loglog(frequencies[ind], V_fb[ind, i]/V_hv[ind, i], colors[color_index]+'o')
                color_index+=1
        color_index = 0
        for i in range(Z_1.shape[1]):
            ind = mlab.find(np.logical_and(V_hv[:, i], V_fb[:, i]))
            if len(ind):
                R_fb = p1[0+i]
                C_fb = p1[len(calibration.R_fb)+i]
                if self.plugin.control_board.calibration.hw_version.major == 1:
                    a.plot(frequencies[ind], R_fb*C_device*2*np.pi*frequencies[ind]/ \
                             np.sqrt(1+np.square(2*np.pi*R_fb*(C_fb+C_device)*frequencies[ind])), colors[i]+'--')
                else:
                    a.plot(frequencies[ind], R_fb*C_device*2*np.pi*frequencies[ind]/ \
                             np.sqrt(1+np.square(2*np.pi*R_fb*C_fb*frequencies[ind])), colors[color_index]+'--')
                color_index+=1
        a.legend(legend)
        a.set_xlabel('Frequency (Hz)')
        a.set_ylabel('V$_{fb}$/V$_{hv}$')
        a.set_title('V$_{fb}$/V$_{hv}$ for a %.1fpF load' % (C_device*1e12))
        canvas.draw()

        # write new calibration parameters to the control board
        for i in range(0, len(calibration.R_fb)):
            self.plugin.control_board.set_series_resistor_index(1,i)
            self.plugin.control_board.set_series_resistance(1, p1[i])
            self.plugin.control_board.set_series_capacitance(1,p1[len(calibration.R_fb)+i])
        # reconnect to update settings
        self.plugin.control_board.connect()
        
    def device_impedance(self, p0, V_hv, V_fb, frequencies):
        R_fb = []
        for i in range(0, len(p0)/2):
            R_fb.append((p0[i]*np.ones(V_fb.shape[0])).tolist())
        R_fb = np.array(R_fb).transpose()
        C_fb = []
        for i in range(len(p0)/2, len(p0)):
            C_fb.append((p0[i]*np.ones(V_fb.shape[0])).tolist())
        C_fb = np.array(C_fb).transpose()
        f = np.tile(np.reshape(frequencies, (len(frequencies), 1)),
                    (1, V_fb.shape[1]))
        if self.plugin.control_board.calibration.hw_version.major == 1:
            return R_fb/np.sqrt(1+np.square(2*np.pi*R_fb*C_fb*f))*(V_hv/V_fb-1)
        else:
            return R_fb/np.sqrt(1+np.square(2*np.pi*R_fb*C_fb*f))*(V_hv/V_fb)

    def process_hv_calibration(self, results):
        hardware_version = utility.Version.fromstring(
            self.plugin.control_board.hardware_version()
        )
        input_voltage = np.array(results['input_voltage'])
        frequencies = np.array(results['frequencies'])
        hv_measurements = np.array(results['hv_measurements'])/1023.0*5-2.5
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
        f = lambda p, x, R1: np.abs(1/(R1/p[1]+1+R1*2*np.pi*p[0]*complex(0,1)*x))
        if hardware_version.major == 2:
            f = lambda p, x, R1: np.abs(1/(R1/p[1]+R1*2*np.pi*p[0]*complex(0,1)*x))
        e = lambda p, x, y, R1: f(p, x, R1) - y
        fit_params = []
    
        canvas, a = self.create_plot('HV attenuation')
        colors = ['b', 'r', 'g']
        legend = []
        for i in range(0, np.size(hv_rms,0)):
            ind = mlab.find(hv_rms[i,:]>.1)
            p0 = [self.plugin.control_board.calibration.C_hv[i],
                  self.plugin.control_board.calibration.R_hv[i]
            ]
            a.loglog(frequencies[ind], f(p0, frequencies[ind], R1), colors[i]+'--')
            legend.append('R$_{%d}$ (previous fit)' % i)

            if 'voltages' in results:
                voltages = np.array(results['voltages'])
                T=attenuation[i,ind]
                p1, success = optimize.leastsq(e, p0, args=(frequencies[ind], T, R1))
                fit_params.append(p1)
                a.loglog(frequencies[ind], f(p1, frequencies[ind], R1), colors[i]+'-')
                legend.append('R$_{%d}$ (new fit)' % i)

                a.plot(frequencies, attenuation[i], colors[i]+'o')
                legend.append('R$_{%d}$ (scope measurements)' % i)

                # update control board calibration
                self.plugin.control_board.set_series_resistor_index(0,i)
                self.plugin.control_board.set_series_resistance(0, abs(p1[1]))
                self.plugin.control_board.set_series_capacitance(0, abs(p1[0]))
                # reconnnect to update calibration data
                self.plugin.control_board.connect()
            else: # control board measurements
                fit_params.append(p0)
                a.plot(frequencies, attenuation[i], colors[i]+'o')
                legend.append('R$_{%d}$ (CB measurements)' % i)
                
        a.legend(legend)
        a.set_xlabel('Frequency (Hz)')
        a.set_ylabel('Attenuation')
        a.set_title('HV attenuation')
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
