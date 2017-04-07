import os
import re
import subprocess as sp
import sys

from paver.easy import task, needs, path, sh
from paver.setuputils import setup
import conda_helpers as ch
import path_helpers as ph
import platformio_helpers as pioh

# add the current directory as the first listing on the python path
# so that we import the correct version.py
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
import version

DEFAULT_ARDUINO_BOARDS = ['mega2560']

setup(name='dmf-control-board-firmware',
      version=version.getVersion(),
      description='Arduino-based DMF control board firmware and Python API.',
      author='Ryan Fobel and Christian Fobel',
      author_email='ryan@fobel.net and christian@fobel.net',
      url='https://github.com/wheeler-microfluidics/dmf-control-board-firmware',
      license='GPLv2',
      packages=['dmf_control_board_firmware'],
      include_package_data=True,
      install_requires=['decorator', 'functools32', 'matplotlib',
                        'microdrop-utility', 'scipy', 'serial_device>=0.4',
                        'svg-model>=0.5.post20', 'sympy', 'tables',
                        'wheeler.base-node>=0.3.post2', 'pandas>=0.17',
                        'arrow'],
      extras_require={'build': ['arduino-scons>=v0.1.post11', 'SCons>=2.4.1']})


@task
def create_config():
    def get_version_string():
        version = sp.check_output('git describe', shell=True).strip()
        branch = sp.check_output('git rev-parse --abbrev-ref HEAD',
                                 shell=True).strip()
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
    sketch_directory = path('src')
    source_data = sketch_directory.joinpath('Config.h.skeleton').bytes()
    config_data = source_data.replace('#define ___SOFTWARE_VERSION___ "0.1.0"',
                                      '#define ___SOFTWARE_VERSION___ "{}"'
                                      .format(get_version_string()))
    sketch_directory.joinpath('Config.h').write_bytes(config_data)


@task
def nosetests():
    nose_options = '-v'
    sh('nosetests %s' % nose_options)


@task
@needs('create_config')
def build_firmware():
    sp.call(['pio', 'run'])


@task
@needs('generate_setup', 'minilib', 'build_firmware', 'nosetests',
       'setuptools.command.sdist')
def sdist():
    """Overrides sdist to make sure that our setup.py is generated."""
    pass


@task
@needs('generate_setup', 'minilib', 'build_firmware', 'nosetests',
       'setuptools.command.bdist_wheel')
def bdist_wheel():
    """Overrides bdist_wheel to make sure that our setup.py is generated."""
    pass


@task
def develop_link(options, info):
    '''
    Prepare development environment.

    Perform the following steps:

     - Uninstall ``dmf_control_board_firmware`` if installed as Conda package.
     - Install build and run-time Conda dependencies.
     - Link working ``.pioenvs`` directory into Conda ``Library`` directory to
       make development versions of compiled firmware binaries available to
       Python API.
     - Link ``dmf_control_board_firmware`` Python package into site packages
       directory.

    See Also
    --------
    :func:`develop_unlink`
    '''
    project_dir = ph.path(__file__).realpath().parent

    # Uninstall ``dmf_control_board_firmware`` if installed as Conda package.
    info('Check if Conda package is installed...')
    version_info = ch.conda_version_info('dmf-control-board-firmware')
    if version_info.get('installed') is not None:
        info('Uninstall `dmf-control-board-firmware` package...')
        ch.conda_exec('uninstall', '-y', 'dmf-control-board-firmware',
                      verbose=True)
    else:
        info('`dmf-control-board-firmware` package is not installed.')

    # Install build and run-time Conda dependencies.
    info('Install build and run-time Conda dependencies...')
    recipe_dir = project_dir.joinpath('.conda-recipe').realpath()
    ch.conda_exec('install', '-y', '-n', 'root', 'conda-build', verbose=True)
    ch.development_setup(recipe_dir, verbose=True)

    # Link working ``.pioenvs`` directory into Conda ``Library`` directory.
    info('Link working firmware directories into Conda environment.')
    pio_bin_dir = pioh.conda_bin_path()

    fw_bin_dir = pio_bin_dir.joinpath('dmf-control-board-firmware')

    if not fw_bin_dir.exists():
        project_dir.joinpath('.pioenvs').junction(fw_bin_dir)

    fw_config_ini = fw_bin_dir.joinpath('platformio.ini')
    if not fw_config_ini.exists():
        project_dir.joinpath('platformio.ini').link(fw_config_ini)

    # Link ``dmf_control_board_firmware`` Python package `conda.pth` in site
    # packages directory.
    info('Link working Python directory into Conda environment...')
    ch.conda_exec('develop', project_dir, verbose=True)
    info(72 * '-' + '\nFinished')


@task
def develop_unlink(options, info):
    '''
    Prepare development environment.

    Perform the following steps:

     - Unlink working ``.pioenvs`` directory into Conda ``Library`` directory.
     - Unlink ``dmf_control_board_firmware`` Python package from site packages
       directory.

    See Also
    --------
    :func:`develop_link`
    '''
    project_dir = ph.path(__file__).realpath().parent

    # Unlink working ``.pioenvs`` directory into Conda ``Library`` directory.
    info('Unlink working firmware directories from Conda environment.')
    pio_bin_dir = pioh.conda_bin_path()
    fw_bin_dir = pio_bin_dir.joinpath('dmf-control-board-firmware')

    if fw_bin_dir.exists():
        fw_config_ini = fw_bin_dir.joinpath('platformio.ini')
        if fw_config_ini.exists():
            fw_config_ini.unlink()
        fw_bin_dir.unlink()

    # Remove link to ``dmf_control_board_firmware`` Python package in
    # `conda.pth` in site packages directory.
    info('Unlink working Python directory from Conda environment...')
    ch.conda_exec('develop', '-u', project_dir, verbose=True)
    info(72 * '-' + '\nFinished')
