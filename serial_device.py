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
import itertools
from time import sleep

from path import path


class ConnectionError(Exception):
    pass


class SerialDevice(object):
    def __init__(self):
        self.port = None
    
    @classmethod
    def get_serial_ports(cls):
        if os.name == 'nt':
            ports = cls._get_serial_ports_windows()
        else:
            ports = itertools.chain(path('/dev').walk('ttyUSB*'),
                            path('/dev').walk('ttyACM*'))
        # sort list alphabetically
        ports_ = [port for port in ports]
        ports_.sort()
        for port in ports_:
            yield port

    @classmethod
    def _get_serial_ports_windows(cls):
        """ Uses the Win32 registry to return a iterator of serial 
            (COM) ports existing on this computer.

            See http://stackoverflow.com/questions/1205383/listing-serial-com-ports-on-windows
        """
        import _winreg as winreg

        reg_path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        try:
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, reg_path)
        except WindowsError:
            # No serial ports. Return empty generator.
            return

        for i in itertools.count():
            try:
                val = winreg.EnumValue(key, i)
                yield str(val[1])
            except EnvironmentError:
                break

    def get_port(self):
        self.port = None

        for test_port in self.get_serial_ports():
            if self.test_connection(test_port):
                self.port = test_port
                break
            sleep(0.1)

        if self.port is None:
            raise ConnectionError('Could not connect to serial device.')

        return self.port

    def test_connection(self, port):
        raise NotImplementedError
