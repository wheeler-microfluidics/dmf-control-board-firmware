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
from cPickle import dumps, loads

import gtk
import numpy as np
import matplotlib
if os.name=='nt':
    matplotlib.rc('font', **{'family':'sans-serif','sans-serif':['Arial']})
from matplotlib.figure import Figure
from matplotlib.backends.backend_gtkagg import FigureCanvasGTKAgg as FigureCanvasGTK
from matplotlib.backends.backend_gtkagg import NavigationToolbar2GTKAgg as NavigationToolbar


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


class FeedbackOptionsController():
    def __init__(self, plugin):
        self.plugin = plugin
        self.builder = gtk.Builder()
        self.builder.add_from_file("plugins/dmf_control_board/microdrop/glade/feedback_options.glade")
        self.window = self.builder.get_object("window")
        self.builder.connect_signals(self)
        self.window.set_title("Feedback Options")
        menu_item = gtk.MenuItem("Feedback Options")
        plugin.app.main_window_controller.menu_tools.append(menu_item)
        menu_item.connect("activate", self.on_window_show)
        menu_item.show()

    def update(self):
        options = self.plugin.current_step_options()

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

    def on_button_feedback_enabled_toggled(self, widget, data=None):
        """
        Handler called when the "Feedback enabled" check box is toggled. 
        """
        options = self.plugin.current_step_options()
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
        options = self.plugin.current_step_options()
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
        self.plugin.current_step_options().sampling_time_ms = \
            check_textentry(widget,
                            self.plugin.current_step_options().sampling_time_ms,
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
        self.plugin.current_step_options().n_samples = \
            check_textentry(widget,
                            self.plugin.current_step_options().n_samples,
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
        self.plugin.current_step_options().delay_between_samples_ms = \
            check_textentry(widget,
                            self.plugin.current_step_options().delay_between_samples_ms,
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
        self.plugin.current_step_options().capacitance_threshold = \
            check_textentry(widget,
                            self.plugin.current_step_options().capacitance_threshold,
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
        self.plugin.current_step_options().max_retries = \
            check_textentry(widget,
                            self.plugin.current_step_options().max_retries,
                            int)

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

    def plot(self, axis):
        t = np.array(range(0,self.options.n_samples)) * \
                (self.options.sampling_time_ms + \
                 self.options.delay_between_samples_ms)
        axis.plot(t, self.Z_device)


class FeedbackResultsController():
    def __init__(self, plugin):
        self.plugin = plugin
        self.builder = gtk.Builder()
        self.builder.add_from_file("plugins/dmf_control_board/microdrop/glade/feedback_results.glade")
        self.window = self.builder.get_object("window")
        self.window.set_title("Feedback Results")
        self.builder.connect_signals(self)
        menu_item = gtk.MenuItem("Feedback Results")
        plugin.app.main_window_controller.menu_view.append(menu_item)
        menu_item.connect("activate", self.on_window_show)
        menu_item.show()

        self.figure = Figure()   
        self.canvas = FigureCanvasGTK(self.figure)
        self.axis = self.figure.add_subplot(111)
        self.vbox = self.builder.get_object("vbox1")
        toolbar = NavigationToolbar(self.canvas, self.window)
        self.vbox.pack_start(self.canvas)
        self.vbox.pack_start(toolbar, False, False)

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

    def on_experiment_log_selection_changed(self, data):
        """
        Handler called whenever the experiment log selection changes.

        Parameters:
            data : dictionary of experiment log data for the selected steps
        """
        self.axis.cla()
        self.axis.set_xlabel("time (ms)")
        self.axis.set_ylabel("|Z$_{device}$(f)| ($\Omega$)")
        self.axis.grid(True)
        self.axis.set_title("Impedance")
        legend = []
        for row in data:
            if row.keys().count("FeedbackResults"):
                results = loads(row["FeedbackResults"])
                results.plot(self.axis)
                legend.append("Step %d (%.3f s)" % (row["step"], row["time"]))
        if len(legend):
            self.axis.legend(legend)
        self.figure.subplots_adjust(left=0.17, bottom=0.15)
        self.canvas.draw()
