from pprint import pprint

from paver.easy import task, needs, path, sh, cmdopts, options
from paver.setuputils import setup, find_package_data

import version

dmf_control_board_files = find_package_data(package='dmf_control_board',
                                            where='dmf_control_board',
                                            only_in_packages=False)
pprint(dmf_control_board_files)

DEFAULT_ARDUINO_BOARDS = ['mega2560']

setup(name='wheeler.dmf_control_board',
      version=version.getVersion(),
      description='Arduino-based dmf_control_board firmware and Python API.',
      author='Ryan Fobel',
      author_email='ryan@fobel.net',
      url='http://microfluidics.utoronto.ca/git/dmf_control_board.git',
      license='GPLv2',
      packages=['dmf_control_board'],
      package_data=dmf_control_board_files,
      install_requires=['avr_helpers', 'arduino_helpers'])


@task
def create_config():
    sketch_directory = path('dmf_control_board').joinpath('src',
                                                          'dmf_control_board')
    sketch_directory.joinpath('Config.h.skeleton').copy(sketch_directory
                                                        .joinpath('Config.h'))


@task
@needs('create_config')
@cmdopts([('sconsflags=', 'f', 'Flags to pass to SCons.'),
          ('boards=', 'b', 'Comma-separated list of board names to compile '
           'for (e.g., `mega2560`).')])
def build_firmware():
    scons_flags = getattr(options, 'sconsflags', '')
    boards = [b.strip() for b in getattr(options, 'boards', '').split(',')
              if b.strip()]
    if not boards:
        boards = DEFAULT_ARDUINO_BOARDS
    for board in boards:
        # Compile firmware once for each specified board.
        sh('scons %s ARDUINO_BOARD="%s"' % (scons_flags, board))


@task
@needs('generate_setup', 'minilib', 'build_firmware',
       'setuptools.command.sdist')
def sdist():
    """Overrides sdist to make sure that our setup.py is generated."""
    pass
