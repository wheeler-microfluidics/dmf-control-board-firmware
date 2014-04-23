from pprint import pprint

from paver.easy import task, needs, path, sh, cmdopts, options
from paver.setuputils import setup, find_package_data, setuptools

import version

dmf_control_board_files = find_package_data(package='dmf_control_board',
                                            where='dmf_control_board',
                                            only_in_packages=False)
pprint(dmf_control_board_files)

setup(name='wheeler.dmf_control_board',
      version=version.getVersion(),
      description='Arduino-based dmf_control_board firmware and Python API.',
      author='Ryan Fobel',
      author_email='ryan@fobel.net',
      url='http://microfluidics.utoronto.ca/git/dmf_control_board.git',
      license='GPLv2',
      packages=['dmf_control_board', 'dmf_control_board.avr'],
      package_data=dmf_control_board_files)


@task
def create_config():
    sketch_directory = path('dmf_control_board').joinpath('src', 'dmf_control_board')
    sketch_directory.joinpath('Config.h.skeleton').copy(sketch_directory
                                                        .joinpath('Config.h'))


@task
@needs('create_config')
@cmdopts([
    ('sconsflags=', 'f', 'Flags to pass to SCons.')
])
def build_firmwares():
    sh('scons %s' % getattr(options, 'sconsflags', ''))


@task
@needs('generate_setup', 'minilib', 'build_firmwares', 'setuptools.command.sdist')
def sdist():
    """Overrides sdist to make sure that our setup.py is generated."""
    pass
