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

import threading
import time
import math
from copy import deepcopy
try:
    from cPickle import dumps, loads
except ImportError:
    from pickle import dumps, loads

import gtk
import numpy as np

import utility
try:
    from ...dmf_control_board import *
    from ...dmf_control_board.microdrop.feedback import *
except:
    # Raise the exception(s) if we're running the program (these exceptions
    # are expected when generating documentation with doxygen, so in that case
    # we can safely ignore them).
    if utility.PROGRAM_LAUNCHED:
        raise

from logger import logger
from plugin_manager import IPlugin, IWaveformGenerator, SingletonPlugin, \
    implements, emit_signal, PluginGlobals
from app_context import get_app
from fields import Field


class WaitForFeedbackMeasurement(threading.Thread):
    def __init__(self, control_board, state, feedback_options):
        self.control_board = control_board
        self.state = state
        self.feedback_options = feedback_options
        self.results = None
        threading.Thread.__init__(self)
  
    def run(self):
        self.results = self.control_board.measure_impedance(
                            self.feedback_options.sampling_time_ms,
                            self.feedback_options.n_samples,
                            self.feedback_options.delay_between_samples_ms,
                            self.state)

PluginGlobals.push_env('microdrop')


class DmfControlBoardOptions(object):
    def __init__(self, voltage=100,
                 frequency=1e3,
                 feedback_options=None):
        if feedback_options is None:
            self.feedback_options = FeedbackOptions()
        else:
            self.feedback_options = feedback_options
        self.voltage = voltage
        self.frequency = frequency


class DmfControlBoardPlugin(SingletonPlugin):
    """
    This class is automatically registered with the PluginManager.
    """
    implements(IPlugin)
    implements(IWaveformGenerator)

    _fields = [Field('voltage', type=float, default=100),
                Field('frequency', type=float, default=1e3)]

    def __init__(self):
        self.control_board = DmfControlBoard()
        self.name = "wheelerlab.dmf_control_board_" + \
            self.control_board.host_hardware_version()        
        self.version = self.control_board.host_software_version()
        self.url = self.control_board.host_url()
        self.steps = [] # list of steps in the protocol
        self.current_state = FeedbackOptions()
        self.feedback_options_controller = None
        self.feedback_results_controller = None
        self.initialized = False

    def on_app_init(self):
        """
        Handler called once when the Microdrop application starts.
        """
        if not self.initialized:
            app = get_app()
            menu_item = gtk.MenuItem("Flash DMF control board firmware")
            app.main_window_controller.menu_tools.append(menu_item)
            menu_item.connect("activate", self.on_flash_firmware)
            menu_item.show()
            
            self.feedback_options_controller = FeedbackOptionsController(self)
            self.feedback_results_controller = FeedbackResultsController(self)
            self.initialized = True
            self.check_device_name_and_version()

    def check_device_name_and_version(self):
        app = get_app()
        try:
            app.control_board = self.control_board
            self.control_board.connect()
            name = self.control_board.name()
            hardware_version = self.control_board.hardware_version()
            host_hardware_version = self.control_board.host_hardware_version()

            if name != "Arduino DMF Controller":
                raise Exception("Device is not an Arduino DMF Controller")
            
            if hardware_version != host_hardware_version:
                raise Exception("The currently installed DMF control board "
                                "plugin is designed for hardware version %s, "
                                "however the connected device is version %s."
                                % (host_hardware_version, hardware_version))
            
            host_software_version = self.control_board.host_software_version()
            remote_software_version = self.control_board.software_version()

            # reflash the firmware if it is not the right version
            if host_software_version !=  remote_software_version:
                response = app.main_window_controller.question("The "
                    "control board firmware version (%s) does not match the "
                    "driver version (%s). Update firmware?" %
                    (remote_software_version, host_software_version),
                    "Update firmware?")
                if response == gtk.RESPONSE_YES:
                    self.on_flash_firmware()
        except Exception, why:
            logger.warning("%s" % why)
        
        self.update_connection_status()
        
    def on_flash_firmware(self, widget=None, data=None):
        app = get_app()
        try:
            self.control_board.flash_firmware()
            app.main_window_controller.info("Firmware updated "
                                                 "successfully.",
                                                 "Firmware update")
        except Exception, why:
            logger.error("Problem flashing firmware. "
                                                  "%s" % why,
                                                  "Firmware update")
        self.check_device_name_and_version()

    def update_connection_status(self):
        app = get_app()
        connection_status = "Not connected"
        if self.control_board.connected():
            try:
                name = self.control_board.name()
                version = self.control_board.hardware_version()
                firmware = self.control_board.software_version()
                connection_status = name + " v" + version + "\n\tFirmware: " + str(firmware)
            except:
                pass

        app.main_window_controller.label_connection_status. \
            set_text(connection_status)

    def get_default_options(self):
        return DmfControlBoardOptions()

    def get_step_options(self, step=None):
        """
        Return a FeedbackOptions object for the current step in the protocol.
        If none exists yet, create a new one.
        """
        app = get_app()
        if step is None:
            step = app.protocol.current_step_number
        
        options = app.protocol.current_step().get_data(self.name)
        if options is None:
            # No data is registered for this plugin (for this step).
            options = self.get_default_options()
            app.protocol.current_step().set_data(self.name, options)
        return options

    def get_actuated_area(self):
        area = 0
        app = get_app()
        state_of_all_channels = app.protocol.state_of_all_channels()        
        for id, electrode in app.dmf_device.electrodes.iteritems():
            channels = app.dmf_device.electrodes[id].channels
            if channels:
                # get the state(s) of the channel(s) connected to this electrode
                states = state_of_all_channels[channels]
                if len(np.nonzero(states>0)[0]):
                    area += electrode.area()*app.dmf_device.scale
        return area

    def on_protocol_update(self, data):
        """
        Handler called whenever the current protocol step changes.
        """
        app = get_app()
        self.feedback_options_controller.update()
        options = self.get_step_options()
        step = app.protocol.current_step()
        dmf_options = step.get_data('microdrop.gui.dmf_device_controller')
        logger.debug('[DmfControlBoardPlugin] options=%s dmf_options=%s' % (options, dmf_options))
        feedback_options = options.feedback_options
        self.current_state.feedback_enabled = feedback_options.feedback_enabled

        if self.control_board.connected() and \
            (app.realtime_mode or app.running):
            state = dmf_options.state_of_channels
            max_channels = self.control_board.number_of_channels() 
            if len(state) >  max_channels:
                state = state[0:max_channels]
            elif len(state) < max_channels:
                state = np.concatenate([state,
                        np.zeros(max_channels - len(state), int)])
            else:
                assert(len(state) == max_channels)

            if feedback_options.feedback_enabled:
                # calculate the total area of actuated electrodes
                area =  self.get_actuated_area()
                
                if feedback_options.action.__class__ == RetryAction:
                    if data.keys().count("attempt") == 0:
                        attempt = 0
                    else:
                        attempt = data["attempt"]

                    if attempt <= feedback_options.action.max_repeats:
                        voltage = float(options.voltage +\
                            feedback_options.action.increase_voltage * attempt)
                        frequency = \
                            float(options.frequency)
                        emit_signal("set_frequency", frequency,
                            interface=IWaveformGenerator)
                        emit_signal("set_voltage", voltage,
                            interface=IWaveformGenerator)
                        (V_hv, hv_resistor, V_fb, fb_resistor) = \
                            self.measure_impedance(state, options)
                        results = FeedbackResults(options,
                            V_hv, hv_resistor,
                            V_fb, fb_resistor,
                            area,
                            frequency,
                            voltage)
                        logger.info("V_total=%s" % results.V_total())
                        logger.info("Z_device=%s" % results.Z_device())                        
                        data["FeedbackResults"] = dumps(results)
                        if max(results.capacitance())/area < \
                            options.action.capacitance_threshold:
                            logger.info('step=%d: attempt=%d, max(C)/A=%.1e F/mm^2. Repeat' % \
                                (self.app.protocol.current_step_number,
                                 attempt, max(results.capacitance())/area))
                            # signal that the step should be repeated
                            return 'Repeat'
                        else:
                            logger.info('step=%d: attempt=%d, max(C)/A=%.1e F/mm^2. OK' % \
                                (self.app.protocol.current_step_number,
                                 attempt, max(results.capacitance())/area))
                            return 'Ok'
                    else:
                        return 'Fail'
                elif feedback_options.action.__class__ == SweepFrequencyAction:
                    frequencies = np.logspace(
                        np.log10(feedback_options.action.start_frequency),
                        np.log10(feedback_options.action.end_frequency),
                        int(feedback_options.action.n_frequency_steps))
                    voltage = float(options.voltage)
                    emit_signal("set_voltage", voltage,
                                interface=IWaveformGenerator)
                    results = SweepFrequencyResults(feedback_options, area, voltage)
                    for frequency in frequencies:
                        emit_signal("set_frequency",
                                    float(frequency),
                                    interface=IWaveformGenerator)
                        (V_hv, hv_resistor, V_fb, fb_resistor) = \
                            self.measure_impedance(state, options)
                        results.add_frequency_step(frequency,
                            V_hv, hv_resistor, V_fb, fb_resistor)
                    data["SweepFrequencyResults"] = dumps(results)
                    logger.info("V_total=%s" % results.V_total())
                    logger.info("Z_device=%s" % results.Z_device())                        
                elif options.action.__class__==SweepVoltageAction:
                    voltages = np.linspace(options.action.start_voltage,
                                           options.action.end_voltage,
                                           options.action.n_voltage_steps)
                    frequency = float(app.protocol.current_step(). \
                        frequency)
                    emit_signal("set_frequency", frequency,
                                interface=IWaveformGenerator)
                    results = SweepVoltageResults(feedback_options, area, frequency)
                    for voltage in voltages:
                        emit_signal("set_voltage", voltage,
                            interface=IWaveformGenerator)
                        (V_hv, hv_resistor, V_fb, fb_resistor) = \
                            self.measure_impedance(state, options)
                        results.add_voltage_step(voltage,
                            V_hv, hv_resistor, V_fb, fb_resistor)
                    data["SweepVoltageResults"] = dumps(results)
                    logger.info("V_total=%s" % results.V_total())
                    logger.info("Z_device=%s" % results.Z_device())                        
            else:
                voltage = float(options.voltage)
                frequency = float(options.frequency)
                emit_signal("set_voltage", voltage,
                            interface=IWaveformGenerator)
                emit_signal("set_frequency",
                            frequency,
                            interface=IWaveformGenerator)
                self.control_board.set_state_of_all_channels(state)
                t = time.time()
                while time.time()-t < \
                    app.protocol.current_step().duration / 1000.0:
                    while gtk.events_pending():
                        gtk.main_iteration()
                    # Sleep for 0.1ms between protocol polling loop iterations.
                    # Without sleeping between iterations, CPU usage spikes to
                    # 100% while protocol is running.  With sleeping, CPU usage
                    # is reduced to <20%.
                    time.sleep(0.0001)
        elif (app.realtime_mode or app.running):
            # Board is not connected
            # run through protocol (even though device is not connected)
            if not app.control_board.connected():
                t = time.time()
                while time.time()-t < app.protocol.current_step().duration / 1000.0:
                    while gtk.events_pending():
                        gtk.main_iteration()
                    # Sleep for 0.1ms between protocol polling loop iterations.
                    # (see above for reasoning)
                    time.sleep(0.0001)

    def measure_impedance(self, state, options):
        thread = WaitForFeedbackMeasurement(self.control_board, state, options)
        thread.start()
        while thread.is_alive():
            while gtk.events_pending():
                gtk.main_iteration()
        return thread.results

    def on_protocol_run(self):
        """
        Handler called when a protocol starts running.
        """
        app = get_app()
        if self.control_board.connected() == False:
            logger.warning("Warning: no control board connected.")
        elif self.control_board.number_of_channels() < app.protocol.n_channels:
            logger.warning("Warning: currently "
                "connected board does not have enough channels for this "
                "protocol.")
    
    def on_experiment_log_selection_changed(self, data):
        """
        Handler called whenever the experiment log selection changes.

        Parameters:
            data : dictionary of experiment log data for the selected steps
        """
        self.feedback_results_controller.on_experiment_log_selection_changed(data)
        
    def set_voltage(self, voltage):
        """
        Set the waveform voltage.
        
        Parameters:
            voltage : RMS voltage
        """
        # TODO: get gain from amplifier object
        gain = 200.0
        self.control_board.set_waveform_voltage(voltage/gain)
        
    def set_frequency(self, frequency):
        """
        Set the waveform frequency.
        
        Parameters:
            frequency : frequency in Hz
        """
        self.control_board.set_waveform_frequency(frequency)

    def get_step_fields(self):
        return self._fields

    def get_step_value(self, name):
        app = get_app()
        if not name in [f.name for f in self._fields]:
            raise KeyError('No field with name %s for plugin %s' % (name, self.name))
        options = app.protocol.current_step().get_data(self.name)
        if options is None:
            field_dict = dict([(f.name, f) for f in self._fields])
            return field_dict[name].default
        return getattr(options, name)


PluginGlobals.pop_env()
