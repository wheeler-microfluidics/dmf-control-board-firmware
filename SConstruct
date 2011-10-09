env = Environment()
env.Append(LIBS=['python2.7',
                    'boost_python',
                    'boost_thread',
                    'boost_filesystem',
                    'boost_system'])
env.Append(CPPPATH=['/usr/include/python2.7'])

Export('env')

if 'arduino' in COMMAND_LINE_TARGETS:
    print 'should build arduino'
print 'COMMAND_LINE_TARGETS:', COMMAND_LINE_TARGETS

# Build host binaries
VariantDir('build/host', 'src', duplicate=0)
SConscript('build/host/SConscript.host')

# Build Arduino binaries
SConscript('src/SConscript.arduino')

# Build documentation
if 'docs' in COMMAND_LINE_TARGETS:
    SConscript('src/SConscript.docs')
    Import('doc')
    Alias('docs', doc)
