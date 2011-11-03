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
from cPickle import dumps, loads
from copy import deepcopy

import gtk
import numpy as np
import matplotlib
matplotlib.use('GTK')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure   
from matplotlib.backends.backend_gtk import FigureCanvasGTK   

from utility import *
from plugin_manager import IPlugin, SingletonPlugin, implements
from plugins.dmf_control_board import *


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


class FeedbackOptions():
    """
    This class stores the feedback options for a single step in the protocol.
    """
    def __init__(self, feedback_enabled=None,
                 sampling_time_ms=None,
                 n_samples=None,
                 delay_between_samples_ms=None,
                 retry=None,
                 capacitance_threshold=None,
                 max_retries=None):
        if feedback_enabled:
            self.feedback_enabled = feedback_enabled
        else:
            self.feedback_enabled = False
        if sampling_time_ms:
            self.sampling_time_ms = sampling_time_ms
        else:
            self.sampling_time_ms = 10
        if n_samples:
            self.n_samples = n_samples
        else:
            self.n_samples = 10
        if delay_between_samples_ms:
            self.delay_between_samples_ms = delay_between_samples_ms
        else:
            self.delay_between_samples_ms = 0
        if retry:
            self.retry = retry
        else:
            self.retry = False
        if capacitance_threshold:
            self.capacitance_threshold = capacitance_threshold
        else:
            self.capacitance_threshold = 0
        if max_retries:
            self.max_retries = max_retries
        else:
            self.max_retries = 3


class FeedbackResults():
    """
    This class stores the impedance results for a single step in the protocol.
    """
    def __init__(self, options, impedance, V_total):
        self.options = options
        self.V_fb = impedance[0::2]
        self.Z_fb = impedance[1::2]
        self.V_total = V_total
        self.Z_device = self.Z_fb*(self.V_total/self.V_fb-1)
        
    def plot(self):
        t = np.array(range(0,self.options.n_samples)) * \
                (self.options.sampling_time_ms + \
                 self.options.delay_between_samples_ms)
        plt.plot(t, self.Z_device)


class FeedbackResultsController():
    def __init__(self, app):
        self.app = app

        """
        self.builder = gtk.Builder()
        self.builder.add_from_file("plugins/dmf_control_board/microdrop/glade/feedback_results.glade")
        self.builder.connect_signals(self)
        self.window = self.builder.get_object("window")
        self.window.set_title("Feedback Results")
        self.figure = Figure(figsize=(6,4), dpi=72)   
        self.axis = self.figure.add_subplot(111)   
        self.axis.set_xlabel("time (ms)")   
        self.axis.set_ylabel("|Z(f)|")   
        self.axis.grid(True)
        self.canvas = FigureCanvasGTK(self.figure)
        self.canvas.show()
        self.builder.get_object("hbox1").pack_start(self.canvas)
        menu_item = gtk.MenuItem("Feedback Results")
        self.app.main_window_controller.menu_tools.append(menu_item)
        menu_item.connect("activate", self.on_window_show)
        menu_item.show()
        """
          
class DmfControlBoardPlugin(SingletonPlugin):
    """
    This class is automatically registered with the PluginManager.
    """
    implements(IPlugin)

    def __init__(self):
        self.control_board = DmfControlBoard()
        self.name = "wheelerlab.arduino_dmf_control_board_" + \
            self.control_board.host_hardware_version()        
        self.version = self.control_board.host_software_version()
        self.url = self.control_board.host_url()
        self.app = None
        self.builder = None
        self.steps = [] # list of steps in the protocol
        self.current_state = FeedbackOptions
        self.feedback_results_controller = None

    def on_app_init(self, app):
        """
        Handler called once when the Microdrop application starts.
        """
        self.app = app
        self.builder = gtk.Builder()
        self.builder.add_from_file("plugins/dmf_control_board/microdrop/glade/feedback_options.glade")
        self.window = self.builder.get_object("window")
        self.builder.connect_signals(self)
        menu_item = gtk.MenuItem("Feedback Options")
        self.app.main_window_controller.menu_tools.append(menu_item)
        menu_item.connect("activate", self.on_window_show)
        menu_item.show()
        self.window.set_title("Feedback Options")
        self.feedback_results_controller = FeedbackResultsController(app)
        
        try:
            self.app.control_board = self.control_board
            self.control_board.connect()
            name = self.control_board.name()
            version = self.control_board.hardware_version()

            # reflash the firmware if it is not the right version
            if self.control_board.host_software_version() != \
                self.control_board.software_version():
                try:
                    self.control_board.flash_firmware()
                except:
                    self.error("Problem flashing firmware")
            firmware = self.control_board.software_version()
            self.app.main_window_controller.label_connection_status.set_text(name + " v" + version + \
                "\n\tFirmware: " + str(firmware))
        except ConnectionError, why:
            print why
        
    def current_step_options(self):
        """
        Return a FeedbackOptions object for the current step in the protocol.
        If none exists yet, create a new one.
        """
        step = self.app.protocol.current_step_number
        if len(self.steps)<=step:
            # initialize the list if it is empty
            if len(self.steps)==0:
                self.steps = [FeedbackOptions()]
            # pad the state list with copies of the last known state
            for i in range(0,step-len(self.steps)+1):
                self.steps.append(deepcopy(self.steps[-1]))
        return self.steps[step]

    def on_window_show(self, widget, data=None):
        """
        Handler called when the user clicks on "Feedback Options" in the "Tools"
        menu.
        """
        self.window.show()

    def on_window_delete_event(self, widget, data=None):
        """
        Handler called when the user closes the "Feedback Options" window. 
        """
        self.window.hide()
        return True

    def on_delete_protocol_step(self):
        """
        Handler called whenever a protocol step is deleted.
        """
        if len(self.steps) > 1:
            del self.steps[self.app.protocol.current_step_number]
        else: # reset first step
            self.steps = [FeedbackOptions()]

    def on_insert_protocol_step(self):
        """
        Handler called whenever a protocol step is inserted.
        """
        self.steps.insert(self.app.protocol.current_step_number,
                          deepcopy(self.current_step_options()))

    def on_protocol_update(self):
        """
        Handler called whenever the current protocol step changes.
        """
        options = self.current_step_options()

        # update the state of the "Feedback enabled" check button        
        button = self.builder.get_object("button_feedback_enabled")
        if options.feedback_enabled != button.get_active():
            button.set_active(options.feedback_enabled)
        else:
            # if the "Feedback enabled" check button state has not changed, we need to
            # call the handler explicitly
            self.on_button_feedback_enabled_toggled(button)

        # update the state of the "Retry until capacitance..." check button        
        button = self.builder.get_object("button_retry")
        if options.retry!= button.get_active():
            button.set_active(options.retry)
        else:
            # if the "Retry until capacitance..." check button state has not
            # changed, we need to call the handler explicitly
            self.on_button_retry_toggled(button)

        # update the sampling time value
        self.builder.get_object("textentry_sampling_time_ms").set_text(
            str(options.sampling_time_ms))

        # update the number of samples value
        self.builder.get_object("textentry_n_samples").set_text(
            str(options.n_samples))

        # update the delay between samples value
        self.builder.get_object("textentry_delay_between_samples_ms").set_text(
            str(options.delay_between_samples_ms))

        # update the capacitance threshold value
        self.builder.get_object("textentry_capacitance_threshold").set_text(
            str(options.capacitance_threshold))

        # update the max retries value
        self.builder.get_object("textentry_max_retries").set_text(
            str(options.max_retries))

        if self.app.realtime_mode or self.app.running:
            if self.control_board.connected():
                self.current_state.feedback_enabled = options.feedback_enabled
                data = {"step":self.app.protocol.current_step_number, 
                        "time":time.time()}
                state = self.app.protocol.current_step().state_of_channels
                max_channels = self.control_board.number_of_channels() 
                if len(state) >  max_channels:
                    state = state[0:max_channels]
                elif len(state) < max_channels:
                    state = np.concatenate([state,
                                            np.zeros(max_channels-len(state),
                                                     int)])
                else:
                    assert(len(state)==max_channels)

                if options.feedback_enabled:
                    thread = WaitForFeedbackMeasurement(self.control_board,
                                                        state,
                                                        options)
                    thread.start()
                    while thread.is_alive():
                        while gtk.events_pending():
                            gtk.main_iteration()
                    results = FeedbackResults(options,
                                              thread.results,
                                              self.app.protocol.current_step().voltage)
                    results.plot()
                    data["FeedbackResults"] = dumps(results)
                else:
                    self.control_board.set_state_of_all_channels(state)
                    t = time.time()
                    while time.time()-t < \
                        self.app.protocol.current_step().time/1000.0:
                        while gtk.events_pending():
                            gtk.main_iteration()
                self.app.experiment_log.add_data(
                    self.app.protocol.current_step_number, data)
        
    def on_app_exit(self):
        """
        Handler called just before the Microdrop application exists. 
        """
        pass

    def on_button_feedback_enabled_toggled(self, widget, data=None):
        """
        Handler called when the "Feedback enabled" check box is toggled. 
        """
        options = self.current_step_options()
        options.feedback_enabled = widget.get_active()
        
        self.builder.get_object("textentry_sampling_time_ms").set_sensitive(
            options.feedback_enabled)
        self.builder.get_object("textentry_n_samples").set_sensitive(
            options.feedback_enabled)
        self.builder.get_object("textentry_delay_between_samples_ms"). \
            set_sensitive(options.feedback_enabled)
        self.builder.get_object("button_retry"). \
            set_sensitive(options.feedback_enabled)
        self.builder.get_object("textentry_capacitance_threshold"). \
            set_sensitive(options.feedback_enabled and options.retry)
        self.builder.get_object("textentry_max_retries"). \
            set_sensitive(options.feedback_enabled and options.retry)

    def on_button_retry_toggled(self, widget, data=None):
        """
        Handler called when the "Retry until capacitance..." check box is
        toggled. 
        """
        options = self.current_step_options()
        options.retry = widget.get_active()
        
        self.builder.get_object("textentry_capacitance_threshold"). \
            set_sensitive(options.feedback_enabled and \
                          options.retry)
        self.builder.get_object("textentry_max_retries"). \
            set_sensitive(options.feedback_enabled and \
                          options.retry)

    def on_textentry_sampling_time_ms_focus_out_event(self, widget, event):
        """
        Handler called when the "sampling time" text box loses focus. 
        """
        self.on_textentry_sampling_time_ms_changed(widget)
    
    def on_textentry_sampling_time_ms_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "sampling time" text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_sampling_time_ms_changed(widget)
    
    def on_textentry_sampling_time_ms_changed(self, widget):
        """
        Update the sampling time value for the current step. 
        """
        self.current_step_options().sampling_time_ms = \
            check_textentry(widget,
                            self.current_step_options().sampling_time_ms,
                            int)

    def on_textentry_n_samples_focus_out_event(self, widget, event):
        """
        Handler called when the "number of samples" text box loses focus. 
        """
        self.on_textentry_n_samples_changed(widget)
    
    def on_textentry_n_samples_key_press_event(self, widget, event):
        """
        
        Handler called when the user presses a key within the "number of
        samples" text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_n_samples_changed(widget)
    
    def on_textentry_n_samples_changed(self, widget):
        """
        Update the number of samples value for the current step. 
        """
        self.current_step_options().n_samples = \
            check_textentry(widget,
                            self.current_step_options().n_samples,
                            int)

    def on_textentry_delay_between_samples_ms_focus_out_event(self,
                                                              widget,
                                                              event):
        """
        Handler called when the "delay between samples" text box loses focus. 
        """
        self.on_textentry_delay_between_samples_ms_changed(widget)
    
    def on_textentry_delay_between_samples_ms_key_press_event(self,
                                                              widget,
                                                              event):
        """
        Handler called when the user presses a key within the "delay between
        samples" text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_delay_between_samples_ms_changed(widget)
    
    def on_textentry_delay_between_samples_ms_changed(self, widget):
        """
        Update the delay between samples value for the current step. 
        """
        self.current_step_options().delay_between_samples_ms = \
            check_textentry(widget,
                            self.current_step_options().delay_between_samples_ms,
                            int)
    
    def on_textentry_capacitance_threshold_focus_out_event(self,
                                                           widget,
                                                           event):
        """
        Handler called when the "capacitance threshold" text box loses focus. 
        """
        self.on_textentry_capacitance_threshold_changed(widget)
    
    def on_textentry_capacitance_threshold_key_press_event(self,
                                                           widget,
                                                           event):
        """
        Handler called when the user presses a key within the "capacitance
        threshold" text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_capacitance_threshold_changed(widget)
    
    def on_textentry_capacitance_threshold_changed(self, widget):
        """
        Update the capacitance threshold value for the current step. 
        """
        self.current_step_options().capacitance_threshold = \
            check_textentry(widget,
                            self.current_step_options().capacitance_threshold,
                            float)
    
    def on_textentry_max_retries_focus_out_event(self, widget, event):
        """
        Handler called when the "max retries" text box loses focus. 
        """
        self.on_textentry_max_retries_changed(widget)
    
    def on_textentry_max_retries_key_press_event(self, widget, event):
        """
        Handler called when the user presses a key within the "max retries"
        text box. 
        """
        if event.keyval == 65293: # user pressed enter
            self.on_textentry_max_retries_changed(widget)
    
    def on_textentry_max_retries_changed(self, widget):
        """
        Update the max retries value for the current step. 
        """
        self.current_step_options().max_retries = \
            check_textentry(widget,
                            self.current_step_options().max_retries,
                            int)
    
    def on_protocol_save(self):
        """
        Handler called when a protocol is saved.
        """
        self.app.protocol.plugin_data[self.name] = (self.version, dumps(self.steps))
    
    def on_protocol_load(self, version, data):
        """
        Handler called when a protocol is loaded.
        """
        self.steps = loads(data)
    
    def on_protocol_run(self):
        """
        Handler called when a protocol starts running.
        """
        pass
        #TODO
    
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
        pass

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