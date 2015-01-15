import logging

import pandas as pd
import time
import warnings

try:
    import visa
    resource_manager = visa.ResourceManager()
    resource_manager.list_resources()
    addresses = [a for a in resource_manager.list_resources()
	     if a.startswith('USB')]
    VISA_AVAILABLE = len(addresses) > 0
    logging.info('Imported `visa`: %s addresses available' % len(addresses))
except Exception, e:
    VISA_AVAILABLE = False
    logging.info('Could not import `visa`: %s' % e)


class AgilentOscope(object):
    def __init__(self, address=None):
        self.rm = visa.ResourceManager()
        if address is None:
            # Only consider USB devices, since any COM port appears
            # to be recognized as a NI instrument *even when it isn't*.
            address = [a for a in self.rm.list_resources()
                       if a.startswith('USB')][0]
        self.device = self.rm.get_instrument(address)

    def read_ac_vrms(self):
        self.device.write("AUTOSCALE")

        def get_V():
            self.device.write("MEASURE:VRMS? DISPLAY,AC")
            for i in xrange(5):
                try:
                    return float(self.device.read())
                except visa.VisaIOError:
                    pass
            raise

        return pd.Series([get_V() for i in xrange(5)]).median()

# `VisaIOError`:
#  - Oscilloscope unplugged while running without restarting.
#  - Oscilloscope not plugged in after reboot.


def get_oscope_reading():
    import PyZenity

    response = None
    while response is None:
        response = PyZenity.GetText()
    return float(response)


def check_text_entry_dialog(*args, **kwargs):
    from pygtkhelpers.ui.extra_dialogs import text_entry_dialog

    validate = kwargs.pop('validate', lambda x: True)
    while True:
        response = text_entry_dialog(*args, **kwargs)
        # Cancel calibration
        if response is None:
            return
        # Check that it is a valid voltage
        elif validate(response):
            return response


def read_oscope():
    title = 'Feedback calibration wizard'
    question = 'What is the current RMS output voltage?'

    def is_float(v):
        try:
            float(v)
            return True
        except:
            return False

    response = check_text_entry_dialog(question, title=title,
                                       validate=is_float)
    if response is None:
        raise StopIteration
    else:
        return float(response)
