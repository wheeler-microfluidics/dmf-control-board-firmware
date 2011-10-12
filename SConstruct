import os
import warnings

env = Environment()

print 'COMMAND_LINE_TARGETS:', COMMAND_LINE_TARGETS


if os.name == 'nt':
    # Initialize ENV with OS environment.  Without this, PATH is not set
    # correctly, leading to doxygen not being found in Windows.
    env['ENV'] = os.environ
    env['LIBPREFIX'] = ''
    env.Append(LIBS=['python27',
                    'ws2_32',
                    'boost_python_mt',
                    'boost_thread_mt',
                    'boost_filesystem_mt',
                    'boost_iostreams_mt',
                    'boost_system_mt'])
    env.Append(CPPPATH=['C:/Python27/include', 'C:/boost'])
    env.Append(LIBPATH=['C:/Python27/libs', 'C:/boost/stage/lib'])

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
                    'python2.7'])
    env.Append(CPPPATH=['/usr/include/python2.7'])

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
