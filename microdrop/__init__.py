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
from cPickle import dumps, loads
from copy import deepcopy

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

class DmfControlBoardPlugin(SingletonPlugin):
    """
    This class is automatically registered with the PluginManager.
    """
    implements(IPlugin)
    implements(IWaveformGenerator)

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

    def current_step_options(self):
        """
        Return a FeedbackOptions object for the current step in the protocol.
        If none exists yet, create a new one.
        """
        app = get_app()
        step = app.protocol.current_step_number
        if len(self.steps)<=step:
            # initialize the list if it is empty
            if len(self.steps) == 0:
                self.steps = [FeedbackOptions()]
            # pad the state list with copies of the last known state
            for i in range(0,step-len(self.steps)+1):
                self.steps.append(deepcopy(self.steps[-1]))
        return self.steps[step]

    def on_delete_protocol_step(self):
        """
        Handler called whenever a protocol step is deleted.
        """
        app = get_app()
        if len(self.steps) > 1:
            del self.steps[app.protocol.current_step_number]
        else: # reset first step
            self.steps = [FeedbackOptions()]

    def on_insert_protocol_step(self):
        """
        Handler called whenever a protocol step is inserted.
        """
        app = get_app()
        self.steps.insert(app.protocol.current_step_number,
                          deepcopy(self.current_step_options()))

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
        options = self.current_step_options()
        self.current_state.feedback_enabled = options.feedback_enabled

        if self.control_board.connected() and \
            (app.realtime_mode or app.running):
            state = app.protocol.current_step().state_of_channels
            max_channels = self.control_board.number_of_channels() 
            if len(state) >  max_channels:
                state = state[0:max_channels]
            elif len(state) < max_channels:
                state = np.concatenate([state,
                                        np.zeros(max_channels-len(state),
                                                 int)])
            else:
                assert(len(state) == max_channels)

            if options.feedback_enabled:
                # calculate the total area of actuated electrodes
                area =  self.get_actuated_area()
                
                if options.action.__class__ == RetryAction:
                    if data.keys().count("attempt") == 0:
                        attempt = 0
                    else:
                        attempt = data["attempt"]

                    if attempt <= options.action.max_repeats:
                        voltage = float(app.protocol.current_step().voltage +
                            options.action.increase_voltage*attempt)
                        frequency = \
                            float(app.protocol.current_step().frequency)
                        emit_signal("set_frequency",
                            frequency,
                            interface=IWaveformGenerator)
                        emit_signal("set_voltage", voltage,
                            interface=IWaveformGenerator)
                        impedance = self.measure_impedance(state, options)
                        results = FeedbackResults(options,
                            impedance,
                            area,
                            frequency,
                            voltage)
                        data["FeedbackResults"] = dumps(results)
                        if max(results.capacitance())/area < \
                            options.action.capacitance_threshold:
                            # signal that the step should be repeated
                            return 'Repeat'
                        else:
                            logger.info('attempt=%d, max(C)/A=%.1e F/mm^2' % \
                                (attempt, max(results.capacitance())/area))
                            return 'Ok'
                    else:
                        return 'Fail'
                elif options.action.__class__ == SweepFrequencyAction:
                    frequencies = np.logspace(
                        np.log10(options.action.start_frequency),
                        np.log10(options.action.end_frequency),
                        int(options.action.n_frequency_steps))
                    voltage = float(app.protocol.current_step(). \
                        voltage)
                    emit_signal("set_voltage", voltage,
                                interface=IWaveformGenerator)
                    results = SweepFrequencyResults(options, area, voltage)
                    for frequency in frequencies:
                        emit_signal("set_frequency",
                                    float(frequency),
                                    interface=IWaveformGenerator)
                        impedance = self.measure_impedance(state, options)
                        results.add_frequency_step(frequency, impedance)
                    data["SweepFrequencyResults"] = dumps(results)
                elif options.action.__class__ == SweepVoltageAction:
                    voltages = np.linspace(options.action.start_voltage,
                                           options.action.end_voltage,
                                           options.action.n_voltage_steps)
                    frequency = float(app.protocol.current_step(). \
                        frequency)
                    emit_signal("set_frequency", frequency,
                                interface=IWaveformGenerator)
                    results = SweepVoltageResults(options, area, frequency)
                    for voltage in voltages:
                        emit_signal("set_voltage", voltage,
                            interface=IWaveformGenerator)
                        impedance = self.measure_impedance(state, options)
                        results.add_voltage_step(voltage, impedance)
                    data["SweepVoltageResults"] = dumps(results)
            else:
                voltage = float(app.protocol.current_step().voltage)
                frequency = float(app.protocol.current_step().frequency)
                emit_signal("set_voltage", voltage,
                            interface=IWaveformGenerator)
                emit_signal("set_frequency",
                            frequency,
                            interface=IWaveformGenerator)
                self.control_board.set_state_of_all_channels(state)
                t = time.time()
                while time.time()-t < \
                    app.protocol.current_step().duration/1000.0:
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
                while time.time()-t < app.protocol.current_step().duration/1000.0:
                    while gtk.events_pending():
                        gtk.main_iteration()
                    # Sleep for 0.1ms between protocol polling loop iterations.
                    # (see above for reasoning)
                    time.sleep(0.0001)


    def measure_impedance(self, state, options):
        thread = WaitForFeedbackMeasurement(self.control_board,
                                            state,
                                            options)
        thread.start()
        while thread.is_alive():
            while gtk.events_pending():
                gtk.main_iteration()
        return thread.results

    def on_app_exit(self):
        """
        Handler called just before the Microdrop application exists. 
        """
        pass
    
    def on_protocol_save(self):
        """
        Handler called when a protocol is saved.
        """
        app = get_app()
        app.protocol.plugin_data[self.name] = (self.version, dumps(self.steps))
    
    def on_protocol_load(self, version, data):
        """
        Handler called when a protocol is loaded.
        """
        self.steps = loads(data)
    
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

    
    def on_protocol_pause(self):
        """
        Handler called when a protocol is paused.
        """
        pass
        
    def on_protocol_changed(self, protocol):
        """
        Handler called when the protocol changes (e.g., when a new protocol
        is loaded).
        """
        if len(protocol) == 1:
            self.steps = []

    def on_dmf_device_changed(self, dmf_device):
        """
        Handler called when the DMF device changes (e.g., when a new device
        is loaded).
        """
        pass

    def on_experiment_log_changed(self, id):
        """
        Handler called when the experiment log changes (e.g., when a protocol
        finishes running.
        """
        pass
        
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
        self.control_board.set_waveform_voltage(voltage / 200.)
        
    def set_frequency(self, frequency):
        """
        Set the waveform frequency.
        
        Parameters:
            frequency : frequency in Hz
        """
        self.control_board.set_waveform_frequency(frequency)


PluginGlobals.pop_env()
