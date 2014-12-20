import distutils.sysconfig
from pprint import pprint
import re
import os
import warnings
import sys

import yaml
from path_helpers import path

import auto_config
from get_libs import get_lib
from git_util import GitUtil


def get_version_string():
    version = GitUtil(None).describe()
    branch = GitUtil(None).branch()
    if branch == "master":
        tags = ""
    else:
        tags = "-" + branch
    m = re.search('^v(?P<major>\d+)\.(?P<minor>\d+)(-(?P<micro>\d+))?', version)
    if m.group('micro'):
        micro = m.group('micro')
    else:
        micro = '0'
    return "%s.%s.%s%s" % (m.group('major'), m.group('minor'), micro, tags)


PYTHON_VERSION = "%s.%s" % (sys.version_info[0],
                            sys.version_info[1])

env = Environment()

print 'COMMAND_LINE_TARGETS:', COMMAND_LINE_TARGETS

SOFTWARE_VERSION = get_version_string()
Export('SOFTWARE_VERSION')

HARDWARE_MAJOR_VERSION_DEFAULT = 2
HARDWARE_MAJOR_VERSION = ARGUMENTS.get('HARDWARE_MAJOR_VERSION', HARDWARE_MAJOR_VERSION_DEFAULT)
Export('HARDWARE_MAJOR_VERSION')

HARDWARE_MINOR_VERSION_DEFAULT = 0
HARDWARE_MINOR_VERSION = ARGUMENTS.get('HARDWARE_MINOR_VERSION', HARDWARE_MINOR_VERSION_DEFAULT)
Export('HARDWARE_MINOR_VERSION')

Import('PYTHON_LIB')


if os.name == 'nt':
    Import('BOOST_HOME')
    Import('BOOST_LIB_PATH')
    Import('PYTHON_LIB_PATH')
    Import('PYTHON_INC_PATH')
    print PYTHON_LIB_PATH
    print PYTHON_INC_PATH
    print(BOOST_HOME)
    print(BOOST_LIB_PATH)

    # Initialize ENV with OS environment.  Without this, PATH is not set
    # correctly, leading to doxygen not being found in Windows.
    env = Environment(tools=['mingw'], ENV=os.environ)
    env['LIBPREFIX'] = ''
    lib_path = [PYTHON_LIB_PATH, BOOST_LIB_PATH]

    BUILD_STATIC = True
    if BUILD_STATIC:
        env.Append(LIBS=[get_lib(lib, LIBPATH=lib_path) \
                            for lib in ['libboost_python*-mt-*.dll.a',
                                        'libboost_filesystem*-mt-*.a',
                                        'libboost_thread*-mt-*.a',
                                        'libboost_system*-mt-*.a',]]
                                    + ['ws2_32', PYTHON_LIB])
        env.Append(CPPDEFINES=dict(BOOST_SYSTEM_STATIC_LINK=1, BOOST_THREAD_USE_LIB=1))
    else:
        env.Append(LIBS=[PYTHON_LIB,
                        'boost_filesystem-mt',
                        'boost_thread-mt',
                        'boost_python-mt',
                        'boost_system-mt',
                        'ws2_32'])
    env.Append(CPPPATH=[PYTHON_INC_PATH, BOOST_HOME])
    env.Append(LIBPATH=lib_path)
    env.Append(CPPFLAGS=['-ftemplate-depth-128', '-fno-inline'])

    #  - Copy dlls to the current directory if necessary.
    libs = [get_lib('libboost_python-*-mt-*.dll'),
            get_lib('mingwm10.dll')]
    for lib in libs:
        lib_destination = path('dmf_control_board_firmware').joinpath(lib.name)
        if not lib_destination.exists():
            path(lib).copy(lib_destination)
else:
    env.Append(LIBS=[get_lib(lib) for lib in ['libboost_python.so',
                                              'libboost_thread.so',
                                              'libboost_filesystem.so',
                                              'libboost_system.so']] +
               [PYTHON_LIB])
    env.Append(CPPPATH=[distutils.sysconfig.get_python_inc()])

# # Build host binaries #
Export('env')
SConscript('dmf_control_board_firmware/src/dmf_control_board/SConscript.host',
            variant_dir='build/host', duplicate=0)

# # Build Arduino binaries #
sketch_build_root = path('build/arduino').abspath()
Export('sketch_build_root')
SConscript('dmf_control_board_firmware/src/dmf_control_board/SConscript.arduino')

Import('arduino_hex')
Import('arduino_hexes')
Import('pyext')
Import('build_context')

# # Install compiled firmwares to `firmwares` directory #
package_hexes = []

for k, v in arduino_hexes.iteritems():
    firmware_path = path('dmf_control_board_firmware').joinpath('firmware',
                                                       build_context
                                                       .ARDUINO_BOARD, k)
    package_hexes.append(env.Install(firmware_path, v)[0])
package_pyext = Install('dmf_control_board_firmware', pyext)

# # Build documentation #
if 'docs' in COMMAND_LINE_TARGETS:
    SConscript('dmf_control_board_firmware/src/dmf_control_board/SConscript.docs')
    Import('doc')
    Alias('docs', doc)
