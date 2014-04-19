import os
import glob
import logging

from path import path


def package_path():
    return path(os.path.abspath(os.path.dirname(__file__)))


def get_sketch_directory():
    '''
    Return directory containing the `dmf_control_board` Arduino sketch.
    '''
    return os.path.join(package_path(), 'src')


def get_includes():
    '''
    Return directories containing the `dmf_control_board` Arduino header files.

    Modules that need to compile against `dmf_control_boad` should use this function
    to locate the appropriate include directories.

    Notes
    =====

    For example:

        import dmf_control_board 
        ...
        print ' '.join(['-I%s' % i for i in dmf_control_board.get_includes()])
        ...

    '''
    return [get_sketch_directory()]


def get_sources():
    '''
    Return `dmf_control_board` Arduino source file paths.

    Modules that need to compile against `dmf_control_board` should use this function
    to locate the appropriate source files to compile.

    Notes
    =====

    For example:

        import dmf_control_board 
        ...
        print ' '.join(dmf_control_board.get_sources())
        ...

    '''
    return glob.glob(os.path.join(get_sketch_directory(), '*.c*'))

logger = logging.getLogger()
