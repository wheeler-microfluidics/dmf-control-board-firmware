import distutils.sysconfig
import re
import os
import sys

from path_helpers import path

from get_libs import get_lib
import auto_config
import conda_helpers as ch
import dmf_control_board_firmware as dmf


PYTHON_VERSION = "%s.%s" % (sys.version_info[0],
                            sys.version_info[1])

env = Environment()

print 'COMMAND_LINE_TARGETS:', COMMAND_LINE_TARGETS

SOFTWARE_VERSION = dmf.__version__
Export('SOFTWARE_VERSION')

HARDWARE_MAJOR_VERSION_DEFAULT = 2
HARDWARE_MAJOR_VERSION = ARGUMENTS.get('HARDWARE_MAJOR_VERSION', HARDWARE_MAJOR_VERSION_DEFAULT)
Export('HARDWARE_MAJOR_VERSION')

HARDWARE_MINOR_VERSION_DEFAULT = 0
HARDWARE_MINOR_VERSION = ARGUMENTS.get('HARDWARE_MINOR_VERSION', HARDWARE_MINOR_VERSION_DEFAULT)
Export('HARDWARE_MINOR_VERSION')

Import('PYTHON_LIB')


if os.name == 'nt':
    Import('PYTHON_INC_PATH')
    print PYTHON_INC_PATH

    # Initialize ENV with OS environment.  Without this, PATH is not set
    # correctly, leading to doxygen not being found in Windows.
    mingw_bin = ch.conda_prefix().joinpath('Library', 'tdm-mingw', 'bin')
    os.environ['PATH'] = '{};'.format(mingw_bin) + os.environ['PATH']
    print os.environ['PATH']
    env = Environment(tools=['mingw'], ENV=os.environ)
    env['LIBPREFIX'] = ''
    lib_path = [ch.conda_prefix(),
                ch.conda_prefix().joinpath('Library', 'bin'),
                ch.conda_prefix().joinpath('Library', 'lib')]

    BUILD_STATIC = True
    if BUILD_STATIC:
        env.Append(LIBS=[get_lib(lib, LIBPATH=lib_path)
                         for lib in ['libboost_python*-mt-*.dll.a',
                                     'libboost_filesystem*-mt-*.a',
                                     'libboost_thread*-mt-*.a',
                                     'libboost_system*-mt-*.a',]] +
                   ['ws2_32', PYTHON_LIB])
        env.Append(CPPDEFINES=dict(BOOST_SYSTEM_STATIC_LINK=1,
                                   BOOST_THREAD_USE_LIB=1))
    else:
        env.Append(LIBS=[PYTHON_LIB,
                         'boost_filesystem-mt',
                         'boost_thread-mt',
                         'boost_python-mt',
                         'boost_system-mt',
                         'ws2_32'])
    env.Append(CPPPATH=[PYTHON_INC_PATH])
    env.Append(LIBPATH=lib_path)
    env.Append(CPPFLAGS=['-ftemplate-depth-128', '-fno-inline'])

    #  - Copy dlls to the current directory if necessary.
    libs = [get_lib('libboost_python-*-mt-*.dll', LIBPATH=lib_path)]
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
env.Append(CPPPATH=[ch.conda_prefix().joinpath('Library', 'include')])

# # Build host binary #
Export('env')
SConscript('src/SConscript.host', variant_dir='build/host', duplicate=0)
Import('pyext')
package_pyext = Install('dmf_control_board_firmware', pyext)

# # Build documentation #
if 'docs' in COMMAND_LINE_TARGETS:
    SConscript('src/SConscript.docs')
    Import('doc')
    Alias('docs', doc)
