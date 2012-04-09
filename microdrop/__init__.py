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
from flatland import Element, Dict, List, String, Integer, Boolean, Float, Form
from flatland.validation import ValueAtLeast, ValueAtMost

from logger import logger
from pygtkhelpers.ui.objectlist import PropertyMapper
from gui.protocol_grid_controller import ProtocolGridController
from plugin_helpers import StepOptionsController, AppDataController
from plugin_manager import IPlugin, IWaveformGenerator, IAmplifier, Plugin, \
    implements, PluginGlobals, ScheduleRequest, emit_signal,\
    ExtensionPoint, get_service_instance
from app_context import get_app
from utility.gui import yesno


class AmplifierGainNotCalibrated(Exception):
    pass


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

PluginGlobals.push_env('microdrop.managed')


class DmfControlBoardOptions(object):
    def __init__(self, duration=100,
                voltage=100,
                frequency=1e3,
                feedback_options=None):
        self.duration = duration
        if feedback_options is None:
            self.feedback_options = FeedbackOptions()
        else:
            self.feedback_options = feedback_options
        self.voltage = voltage
        self.frequency = frequency


def format_func(value):
    if value:
        # Green
        #return '#00FF00'
        return True
    else:
        # Yellow
        #return '#FFFF00'
        return False


class DmfControlBoardPlugin(Plugin, AppDataController, StepOptionsController):
    """
    This class is automatically registered with the PluginManager.
    """
    implements(IPlugin)
    implements(IWaveformGenerator)
    implements(IAmplifier)    

    AppFields = Form.of(
        Dict.named('amplifier_gain').using(optional=True,
            properties=dict(show_in_gui=False)).of(
                List.named('frequency').of(Float),
                List.named('gain').of(Float),
        ),
    )

    StepFields = Form.of(
        Integer.named('duration').using(default=100, optional=True,
            validators=[ValueAtLeast(minimum=0), ]),
        Integer.named('voltage').using(default=100, optional=True,
            validators=[ValueAtLeast(minimum=0), ]),
        Integer.named('frequency').using(default=1e3, optional=True,
            validators=[ValueAtLeast(minimum=0), ]),
        Boolean.named('feedback_enabled').using(default=False, optional=True),
        Integer.named('sampling_time_ms').using(default=10, optional=True,
            validators=[ValueAtLeast(minimum=0), ],
            properties={'mappers': [PropertyMapper('sensitive',
                    attr='feedback_enabled'), PropertyMapper('editable',
                            attr='feedback_enabled'), ],
            }),
        Integer.named('n_samples').using(default=10, optional=True,
            validators=[ValueAtLeast(minimum=0), ],
            properties={'mappers': [PropertyMapper('sensitive',
                    attr='feedback_enabled'), PropertyMapper('editable',
                            attr='feedback_enabled'), ],
            }),
        Integer.named('delay_between_samples_ms').using(default=0, optional=True,
            validators=[ValueAtLeast(minimum=0), ],
            properties={'mappers': [PropertyMapper('sensitive',
                    attr='feedback_enabled'), PropertyMapper('editable',
                            attr='feedback_enabled'), ],
            }),
    )
    _feedback_fields = set(['feedback_enabled', 'sampling_time_ms', 'n_samples',
                            'delay_between_samples_ms'])

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
        self.feedback_calibration_controller = None
        self.initialized = False

    def on_plugin_enable(self):
        self.on_app_init()
        if get_app().protocol:
            self.on_step_run()
            pgc = get_service_instance(ProtocolGridController, env='microdrop')
            pgc.update_grid()

    def on_plugin_disable(self):
        if get_app().protocol:
            self.on_step_run()
            pgc = get_service_instance(ProtocolGridController, env='microdrop')
            #pgc.update_gui()

    def on_app_init(self):
        """
        Handler called once when the Microdrop application starts.
        """
        if not self.initialized:
            self.feedback_options_controller = FeedbackOptionsController(self)
            self.feedback_results_controller = FeedbackResultsController(self)
            self.feedback_calibration_controller = \
                FeedbackCalibrationController(self)

            app = get_app()
            menu_item = gtk.MenuItem("DMF control board")
            app.main_window_controller.menu_tools.append(menu_item)
            menu_item.show()
            control_board_menu = gtk.Menu()
            control_board_menu.show()
            menu_item.set_submenu(control_board_menu)
            
            menu_item = gtk.MenuItem("Flash firmware")
            menu_item.connect("activate", self.on_flash_firmware)
            menu_item.show()
            control_board_menu.append(menu_item)
            
            menu_item = gtk.MenuItem("Perform calibration")
            menu_item.connect("activate",
                self.feedback_calibration_controller.on_perform_calibration)
            control_board_menu.append(menu_item)
            menu_item.show()
            
            menu_item = gtk.MenuItem("Load calibration from file")
            menu_item.connect("activate",
                              self.feedback_calibration_controller. \
                                  on_load_calibration_from_file)
            control_board_menu.append(menu_item)
            menu_item.show()
            
            self.initialized = True
            self.check_device_name_and_version()

    def check_device_name_and_version(self):
        try:
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
                response = yesno("The "
                    "control board firmware version (%s) does not match the "
                    "driver version (%s). Update firmware?" %
                    (remote_software_version, host_software_version))
                    #"Update firmware?")
                if response == gtk.RESPONSE_YES:
                    self.on_flash_firmware()
        except Exception, why:
            logger.warning("%s" % why)
        
        self.update_connection_status()
        
    def on_flash_firmware(self, widget=None, data=None):
        app = get_app()
        try:
            self.control_board.flash_firmware()
            logger.info("Firmware updated "
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

    def get_actuated_area(self):
        area = 0
        app = get_app()
        options = app.dmf_device_controller.get_step_options()
        state_of_all_channels = options.state_of_channels
        for id, electrode in app.dmf_device.electrodes.iteritems():
            channels = app.dmf_device.electrodes[id].channels
            if channels:
                # get the state(s) of the channel(s) connected to this electrode
                states = state_of_all_channels[channels]
                if len(np.nonzero(states > 0)[0]):
                    area += electrode.area() * app.dmf_device.scale
        return area

    def on_step_run(self):
        """
        Handler called whenever a step is executed.

        Returns:
            True if the step should be run again (e.g., if a feedback
            plugin wants to signal that the step should be repeated)
        """
        logger.debug('[DmfControlBoardPlugin] on_step_run()')
        app = get_app()
        options = self.get_step_options()
        dmf_options = app.dmf_device_controller.get_step_options()
        logger.debug('[DmfControlBoardPlugin] options=%s dmf_options=%s' % (options, dmf_options))
        feedback_options = options.feedback_options
        self.current_state.feedback_enabled = feedback_options.feedback_enabled

        start_time = time.time()

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
                area = self.get_actuated_area()
                
                if feedback_options.action.__class__ == RetryAction:
                    if 'attempt' not in app.experiment_log.data[-1]['core'].keys():
                        attempt = 0
                    else:
                        attempt = app.experiment_log.data[-1]['core']['attempt']

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
                            self.measure_impedance(state, feedback_options)
                        results = FeedbackResults(feedback_options,
                            V_hv,
                            hv_resistor,
                            V_fb,
                            fb_resistor,
                            area,
                            frequency,
                            voltage,
                            self.control_board.calibration)
                        logger.info("V_total=%s" % results.V_total())
                        logger.info("Z_device=%s" % results.Z_device())                        
                        app.experiment_log.add_data({"FeedbackResults":results},
                                                    self.name)
                        if max(results.capacitance())/area < \
                            feedback_options.action.percent_threshold/100.0 *\
                                RetryAction.capacitance_threshold:
                            logger.info('step=%d: attempt=%d, max(C)/A=%.1e F/mm^2. Repeat' % \
                                (app.protocol.current_step_number,
                                 attempt, max(results.capacitance())/area))
                            # signal that the step should be repeated
                            return 'Repeat'
                        else:
                            logger.info('step=%d: attempt=%d, max(C)/A=%.1e F/mm^2. OK' % \
                                (app.protocol.current_step_number,
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
                    results = SweepFrequencyResults(feedback_options,
                        area,
                        voltage,
                        self.control_board.calibration)
                    for frequency in frequencies:
                        emit_signal("set_frequency",
                                    float(frequency),
                                    interface=IWaveformGenerator)
                        (V_hv, hv_resistor, V_fb, fb_resistor) = \
                            self.measure_impedance(state, feedback_options)
                        results.add_frequency_step(frequency,
                            V_hv, hv_resistor, V_fb, fb_resistor)
                    app.experiment_log.add_data({"SweepFrequencyResults":results},
                                                self.name)
                    logger.info("V_total=%s" % results.V_total())
                    logger.info("Z_device=%s" % results.Z_device())                        
                elif feedback_options.action.__class__==SweepVoltageAction:
                    voltages = np.linspace(feedback_options.action.start_voltage,
                                           feedback_options.action.end_voltage,
                                           feedback_options.action.n_voltage_steps)
                    frequency = float(app.protocol.current_step(). \
                        frequency)
                    emit_signal("set_frequency", frequency,
                                interface=IWaveformGenerator)
                    results = SweepVoltageResults(feedback_options,
                        area,
                        frequency,
                        self.control_board.calibration)
                    for voltage in voltages:
                        emit_signal("set_voltage", voltage,
                                    interface=IWaveformGenerator)
                        (V_hv, hv_resistor, V_fb, fb_resistor) = \
                            self.measure_impedance(state, feedback_options)
                        results.add_voltage_step(voltage,
                            V_hv, hv_resistor, V_fb, fb_resistor)
                    app.experiment_log.add_data({"SweepVoltageResults":results},
                                                self.name)
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
                self.control_board.state_of_all_channels = state

        # if a protocol is running, wait for the specified minimum duration
        if app.running and not app.realtime_mode:
            while time.time() - start_time < options.duration / 1000.0:
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
        elif self.control_board.number_of_channels() <= app.dmf_device.max_channel():
            logger.warning("Warning: currently "
                "connected board does not have enough channels for this "
                "protocol.")
    
    def on_experiment_log_selection_changed(self, data):
        """
        Handler called whenever the experiment log selection changes.

        Parameters:
            data : dictionary of experiment log data for the selected steps
        """
        if self.feedback_results_controller:
            self.feedback_results_controller.on_experiment_log_selection_changed(data)
        
    def set_voltage(self, voltage):
        """
        Set the waveform voltage.
        
        Parameters:
            voltage : RMS voltage
        """
        gain = self.gain(self.control_board.waveform_frequency())
        self.control_board.set_waveform_voltage(voltage/gain)
        
    def set_frequency(self, frequency):
        """
        Set the waveform frequency.
        
        Parameters:
            frequency : frequency in Hz
        """
        self.control_board.set_waveform_frequency(frequency)

    def gain(self, frequency):
        values_dict = get_app_values()
        if 'amplifier_gain' in values_dict:
            frequencies = values_dict['amplifier_gain']['frequency']
            gain = values_dict['amplifier_gain']['gain']
            return np.interp(frequency,
                             frequencies,
                             gain)
        else:
            raise AmplifierGainNotCalibrated("Amplifier gain not calibrated.")

    def get_default_step_options(self):
        return DmfControlBoardOptions()

    def set_step_values(self, values_dict, step_number=None):
        step_number = self.get_step_number(step_number)
        logger.debug('[DmfControlBoardPlugin] set_step[%d]_values(): '\
                    'values_dict=%s' % (step_number, values_dict,))
        el = self.StepFields(value=values_dict)
        if not el.validate():
            raise ValueError('Invalid values: %s' % el.errors)
        options = self.get_step_options(step_number=step_number)
        for name, field in el.iteritems():
            if field.value is None:
                continue
            if name in self._feedback_fields:
                setattr(options.feedback_options, name, field.value)
            else:
                setattr(options, name, field.value)
        emit_signal('on_step_options_changed', [self.name, step_number],
                    interface=IPlugin)

    def get_step_values(self, step_number=None):
        app = get_app()
        if step_number is None:
            step_number = app.protocol.current_step_number
        step = app.protocol.steps[step_number]

        options = step.get_data(self.name)
        if options is None:
            return None

        values = {}
        for name in self.StepFields.field_schema_mapping:
            try:
                value = getattr(options, name)
            except AttributeError:
                value = getattr(options.feedback_options, name)
            values[name] = value
        return values

    def get_step_value(self, name, step_number=None):
        app = get_app()
        if not name in self.StepFields.field_schema_mapping:
            raise KeyError('No field with name %s for plugin %s' % (name, self.name))
        if step_number is None:
            step_number = app.protocol.current_step_number
        step = app.protocol.steps[step_number]

        options = step.get_data(self.name)
        if options is None:
            return None
        try:
            return getattr(options, name)
        except AttributeError:
            return getattr(options.feedback_options, name)

    def on_step_options_changed(self, plugin, step_number):
        app = get_app()
        logger.debug('[DmfControlBoardPlugin] on_step_options_changed():'\
                    '%s step #%d' % (plugin, step_number))
        if self.feedback_options_controller:
            self.feedback_options_controller\
                .on_step_options_changed(plugin, step_number)
        if not app.running and (plugin=='microdrop.gui.dmf_device_controller' or \
                plugin==self.name) and app.protocol.current_step_number==step_number:
            self.on_step_run()

    def on_experiment_log_created(self, log):
        app = get_app()
        data = {}
        if self.control_board.connected():
            data["control board name"] = \
                self.control_board.name()
            data["control board hardware version"] = \
                self.control_board.hardware_version()
            data["control board software version"] = \
                self.control_board.software_version()
        log.add_data(data)

    def get_schedule_requests(self, function_name):
        """
        Returns a list of scheduling requests (i.e., ScheduleRequest
        instances) for the function specified by function_name.
        """
        if function_name == 'on_app_init' or function_name == 'on_plugin_enable':
            return [ScheduleRequest('microdrop.gui.main_window_controller', 'wheelerlab.dmf_control_board_1.2'),
                    ScheduleRequest('microdrop.gui.dmf_device_controller', 'wheelerlab.dmf_control_board_1.2')]
        elif function_name in ['on_step_options_changed']:
            return [ScheduleRequest(self.name, 'microdrop.gui.protocol_grid_controller')]
        elif function_name in ['on_dmf_device_changed']:
            return [ScheduleRequest('wheelerlab.dmf_control_board_1.2', 'microdrop.gui.dmf_device_controller')]
        return []

    def on_experiment_log_created(self, log):
        app = get_app()
        data = {}
        if self.control_board.connected():
            data["control board name"] = \
                self.control_board.name()
            data["control board hardware version"] = \
                self.control_board.hardware_version()
            data["control board software version"] = \
                self.control_board.software_version()
        log.add_data(data)


PluginGlobals.pop_env()
