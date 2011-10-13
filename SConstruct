import os
import warnings
import auto_config
from static_libs import get_static_lib


env = Environment()

print 'COMMAND_LINE_TARGETS:', COMMAND_LINE_TARGETS

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
        env.Append(LIBS=[get_static_lib(lib, LIBPATH=lib_path) \
                            for lib in [PYTHON_LIB,
                                        'boost_filesystem_mt',
                                        'boost_thread_mt',
                                        'boost_python_mt',
                                        'boost_iostreams_mt',
                                        'boost_system_mt',]]
                                    + ['ws2_32'])
        env.Append(CPPDEFINES=dict(BOOST_PYTHON_STATIC_LIB=None, BOOST_SYSTEM_STATIC_LINK=1, BOOST_THREAD_USE_LIB=1))
    else:
        env.Append(LIBS=[PYTHON_LIB,
                        'boost_filesystem_mt',
                        'boost_thread_mt',
                        'boost_python_mt',
                        'boost_iostreams_mt',
                        'boost_system_mt',
                        'ws2_32'])
    env.Append(CPPPATH=[PYTHON_INC_PATH, BOOST_HOME])
    env.Append(LIBPATH=lib_path)

    # Build host binaries

    Export('env')
    VariantDir('build/host', 'src', duplicate=0)
    SConscript('build/host/SConscript.host')

    # Build Arduino binaries
    SConscript('src/SConscript.arduino')
else:
    env.Append(LIBS=[get_static_lib(lib) for lib in ['boost_python',
                    'boost_thread',
                    'boost_filesystem',
                    'boost_system',
                    PYTHON_LIB]])
    env.Append(CPPPATH=['/usr/include/%s' % PYTHON_LIB])

    # Build host binaries

    Export('env')
    VariantDir('build/host', 'src', duplicate=0)
    SConscript('build/host/SConscript.host')

    # Build Arduino binaries
    SConscript('src/SConscript.arduino')

Import('arduino_hex')
Import('pyext')
Install('bin', arduino_hex)
Install('bin', pyext)


# Build documentation
if 'docs' in COMMAND_LINE_TARGETS:
    SConscript('src/SConscript.docs')
    Import('doc')
    Alias('docs', doc)
