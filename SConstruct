import re
import os
import warnings
import sys

import yaml
from path import path

import auto_config
from get_libs import get_lib
from git_util import GitUtil


def get_plugin_version():
    version = GitUtil(None).describe()
    m = re.search('^v(?P<major>\d+)\.(?P<minor>\d+)(-(?P<micro>\d+))?', version)
    if m.group('micro'):
        micro = m.group('micro')
    else:
        micro = '0'
    version_string = "%s.%s.%s" % (m.group('major'),
            m.group('minor'), micro)
    return version_string


PYTHON_VERSION = "%s.%s" % (sys.version_info[0],
                            sys.version_info[1])

env = Environment()

print 'COMMAND_LINE_TARGETS:', COMMAND_LINE_TARGETS

SOFTWARE_VERSION = get_plugin_version()
Export('SOFTWARE_VERSION')

HARDWARE_MAJOR_VERSION = 1
Export('HARDWARE_MAJOR_VERSION')

HARDWARE_MINOR_VERSION_DEFAULT = 3
HARDWARE_MINOR_VERSION = ARGUMENTS.get('HARDWARE_MINOR_VERSION', HARDWARE_MINOR_VERSION_DEFAULT)
Export('HARDWARE_MINOR_VERSION')

Import('PYTHON_LIB')

extra_files = []
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

    # Build host binaries

    Export('env')
    VariantDir('build/host', 'src', duplicate=0)
    SConscript('build/host/SConscript.host')

    # Copy dlls to the current dir if necessary
    libs = [get_lib('libboost_python-*-mt-*.dll'),
            get_lib('mingwm10.dll')]
    for lib in libs:
        if path(lib.name).exists()==False:
            path(lib).copy('.')
        extra_files.append(lib.name)

    # Build Arduino binaries
    SConscript('src/SConscript.arduino')
else:
    env.Append(LIBS=[get_lib(lib) for lib in ['libboost_python.so',
                    'libboost_thread-mt.so',
                    'libboost_filesystem-mt.so',
                    'libboost_system-mt.so']] \
                    + [PYTHON_LIB])
    env.Append(CPPPATH=['/usr/include/%s' % PYTHON_LIB])

    # Build host binaries

    Export('env')
    VariantDir('build/host', 'src', duplicate=0)
    SConscript('build/host/SConscript.host')

    # Build Arduino binaries
    SConscript('src/SConscript.arduino')

Import('arduino_hex')
Import('pyext')
package_hex = Install('.', arduino_hex)
package_pyext = Install('.', pyext)

# Build documentation
if 'docs' in COMMAND_LINE_TARGETS:
    SConscript('src/SConscript.docs')
    Import('doc')
    Alias('docs', doc)


tar_env = Environment(tools = ["default", "disttar"],
        DISTTAR_EXCLUDEDIRS=['.git'],
        DISTTAR_EXCLUDERES=[r'\.sconsign\.dblite'],
        DISTTAR_EXCLUDEEXTS=['.gz', '.pyc', '.tgz', '.swp'])

version_target = Command('version.txt', None,
                        'echo %s > $TARGET' % SOFTWARE_VERSION)
plugin_root = path('.').abspath()
properties_target = plugin_root.joinpath('properties.yml')
properties = {'name': str(plugin_root.name), 'version': SOFTWARE_VERSION}
properties_target.write_bytes(yaml.dump(properties))
archive_name = '%s-%s.tar.gz' % (properties['name'], SOFTWARE_VERSION)

# This will build an archive using what ever DISTTAR_FORMAT that is set.
tar = tar_env.DistTar('%s' % properties['name'], [tar_env.Dir('#')])
renamed_tar = tar_env.Command(tar_env.File(archive_name), None,
        Move(archive_name, tar[0]))
Depends(tar, [package_hex, package_pyext, version_target] + extra_files)
Depends(renamed_tar, tar)
Clean(renamed_tar, tar)

if 'PLUGIN_ARCHIVE_DIR' in os.environ:
    target_archive_dir = os.environ['PLUGIN_ARCHIVE_DIR']
    Install(target_archive_dir, renamed_tar)
