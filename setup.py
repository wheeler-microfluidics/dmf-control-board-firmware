#!/usr/bin/env python

import distutils.core

try:
    from distutils.command.build_py import build_py_2to3 as build_py
except ImportError:
    from distutils.command.build_py import build_py
import version

# Setup script for path

kw = {
    'name': 'dmf_control_board',
    'version': version.getVersion(),
    'description': 'Arduino-based dmf_control_board firmware and Python API.',
    'author': 'Ryan Fobel',
    'author_email': 'ryan@fobel.net',
    'url': 'http://microfluidics.utoronto.ca/git/dmf_control_board.git',
    'license': 'GPLv2',
    'packages': ['dmf_control_board'],
    'cmdclass': dict(build_py=build_py),
    'package_data': {'dmf_control_board': ['src/*', 'driver/*', 'avr/avrdude*',
                                           'avr/*.bat', 'avr/*.dll', 'firmware/*/*',
                                           '*.dll', '*.pyd']}}

# If we're running Python 2.3, add extra information
if hasattr(distutils.core, 'setup_keywords'):
    if 'classifiers' in distutils.core.setup_keywords:
        kw['classifiers'] = ['Development Status :: 3 - Alpha',
                             'Intended Audience :: Developers',
                             'License :: OSI Approved :: GNU General Public '
                             'License (GPLv2)',
                             'Operating System :: POSIX :: Linux',
                             'Programming Language :: Python']


distutils.core.setup(**kw)
