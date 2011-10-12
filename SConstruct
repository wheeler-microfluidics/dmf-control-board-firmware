import os
import warnings

import auto_config


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
    env.Append(LIBS=[PYTHON_LIB,
                    'ws2_32',
                    'boost_python_mt',
                    'boost_thread_mt',
                    'boost_filesystem_mt',
                    'boost_iostreams_mt',
                    'boost_system_mt'])
    env.Append(CPPPATH=[PYTHON_INC_PATH, BOOST_HOME])
    env.Append(LIBPATH=[PYTHON_LIB_PATH, BOOST_LIB_PATH])

    # Build host binaries

    Export('env')
    VariantDir('build/host', 'src', duplicate=0)
    SConscript('build/host/SConscript.host')

    # Build Arduino binaries
    SConscript('src/SConscript.arduino')
else:
    env.Append(LIBS=['boost_python',
                    'boost_thread',
                    'boost_filesystem',
                    'boost_system',
                    PYTHON_LIB])
    env.Append(CPPPATH=['/usr/include/%s' % PYTHON_LIB])

    # Build host binaries

    Export('env')
    VariantDir('build/host', 'src', duplicate=0)
    SConscript('build/host/SConscript.host')

    # Build Arduino binaries
    SConscript('src/SConscript.arduino')


# Build documentation
if 'docs' in COMMAND_LINE_TARGETS:
    SConscript('src/SConscript.docs')
    Import('doc')
    Alias('docs', doc)
