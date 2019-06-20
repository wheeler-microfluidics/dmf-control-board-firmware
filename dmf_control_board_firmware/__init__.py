import logging
from ._version import get_versions

__version__ = get_versions()['version']
del get_versions

try:
    from .core import *
except ImportError as exception:
    logging.error('error importing core module', exc_info=True)
