import os
import sys
import re

from SCons.Script import Export

from path_helpers import path
from find_avrdude import get_arduino_paths


def get_python_path():
    return path(os.__file__).parent


v = sys.version_info
if os.name == 'nt':
    PYTHON_LIB = 'python%(major)s%(minor)s' % dict(major=v[0], minor=v[1])
else:
    PYTHON_LIB = 'python%(major)s.%(minor)s' % dict(major=v[0], minor=v[1])

PYTHON_PATH = get_python_path().parent
PYTHON_LIB_PATH = PYTHON_PATH / path('libs')
PYTHON_INC_PATH = PYTHON_PATH / path('include')
Export('PYTHON_LIB', 'PYTHON_LIB_PATH', 'PYTHON_INC_PATH')

if __name__ == '__main__':
    if os.name == 'nt':
        print get_boost_lib_path()

    print get_python_path()
    arduino_path, avrdude, avrdude_conf = get_arduino_paths()
    print 'found arduino path:', arduino_path
    print 'using newest avrdude:', avrdude
    print 'using avrdude config:', avrdude_conf
