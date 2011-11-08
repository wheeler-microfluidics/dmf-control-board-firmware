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

from utility.path import path


class ConnectionError(Exception):
    pass


class SerialDevice(object):
    def __init__(self):
        self.port = None
    
    def get_port(self):
        self.port = None
        if os.name == 'nt':
            # Windows
            for i in range(0,31):
                test_port = "COM%d" % i
                if self.test_connection(test_port):
                    self.port = test_port
                    break
        else:
            port = None
            # Assume Linux (Ubuntu)...
            for port in path('/dev').walk('ttyUSB*'):
                if self.test_connection(tty):
                    self.port = port
                    break
            # or Ubuntu in a VirtualBox
            if port is None:
                for port in path('/dev').walk('ttyACM*'):
                    if self.test_connection(port):
                        self.port = port
                        break
        if self.port is None:
            raise ConnectionError('could not connect to serial device.')
        return self.port


    def test_connection(self, port):
        raise NotImplementedError
