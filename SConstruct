env = Environment()
env.Append(LIBS=['python2.7',
                    'boost_python',
                    'boost_thread',
                    'boost_filesystem',
                    'boost_system'])
env.Append(CPPPATH=['/usr/include/python2.7'])

Export('env')
VariantDir('build', 'src', duplicate=0)
SConscript('build/SConscript')
