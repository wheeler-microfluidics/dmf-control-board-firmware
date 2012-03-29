import sys
import logging

from path import path

try:
    import utility
    def package_path():
        return utility.base_path() / path('plugins') / \
            path('dmf_control_board')
except ImportError:
    def package_path():
        try:
            script = path(__file__)
        except NameError:
            script = path(sys.argv[0])
        return script.parent

logger = logging.getLogger()

from dmf_control_board import *
