import sys
import logging

from path import path

try:
    from microdrop.app_context import get_app

    def package_path():
        return (path(get_app().config['plugins']['directory'])
                .joinpath('dmf_control_board'))
except ImportError:
    def package_path():
        try:
            script = path(__file__)
        except NameError:
            script = path(sys.argv[0])
        return script.parent

logger = logging.getLogger()
