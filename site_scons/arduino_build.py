# Copyright (C) 2012 by Christian Fobel <christian@fobel.net>

# Based on the scons script for an Arduino sketch at:
# http://code.google.com/p/arscons/
#
# Copyright (C) 2010 by Homin Lee <ff4500@gmail.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.

# You'll need the serial module: http://pypi.python.org/pypi/pyserial

# Basic Usage:
# 1. make a folder which have same name of the sketch (ex. Blink/ for Blik.pde)
# 2. put the sketch and SConstruct(this file) under the folder.
# 3. to make the HEX. do following in the folder.
#     $ scons
# 4. to upload the binary, do following in the folder.
#     $ scons upload

# Thanks to:
# * Ovidiu Predescu <ovidiu@gmail.com> and Lee Pike <leepike@gmail.com>
#     for Mac port and bugfix.
#
# This script tries to determine the port to which you have an Arduino
# attached. If multiple USB serial devices are attached to your
# computer, you'll need to explicitly specify the port to use, like
# this:
#
# $ scons ARDUINO_PORT=/dev/ttyUSB0
#
# To add your own directory containing user libraries, pass EXTRA_LIB
# to scons, like this:
#
# $ scons EXTRA_LIB=<my-extra-library-dir>
#
from glob import glob
import sys
import re
import os
import platform
from itertools import chain

from path import path
from SCons.Environment import Environment
from SCons.Builder import Builder

from find_avrdude import get_arduino_paths


def getUsbTty(rx):
    usb_ttys = glob(rx)
    if len(usb_ttys) == 1: return usb_ttys[0]
    else: return None


def gather_sources(source_root):
    source_root = path(source_root)
    return source_root.files('*.c') + source_root.files('*.cpp') +\
                    source_root.files('*.S')


def get_lib_candidate_list(pde_path, arduino_version):
    '''
    Scan the .pde file to generate a list of included libraries.
    '''
    # Generate list of library headers included in .pde file
    lib_candidates = []
    ptn_lib = re.compile(r'^[ ]*#[ ]*include [<"](.*)\.h[>"]')
    for line in open(pde_path):
        result = ptn_lib.findall(line)
        if result:
            lib_candidates += result

    # Hack. In version 20 of the Arduino IDE, the Ethernet library depends
    # implicitly on the SPI library.
    if arduino_version >= 20 and 'Ethernet' in lib_candidates:
        lib_candidates += ['SPI']
    return lib_candidates


class ArduinoBuildContext(object):
    def __init__(self, scons_arguments, build_root=None):
        AVR_BIN_PREFIX = None
        ARDUINO_HOME, AVRDUDE_BIN, AVRDUDE_CONF = get_arduino_paths()
        print 'found arduino path:', ARDUINO_HOME
        print 'using newest avrdude:', AVRDUDE_BIN
        print 'using avrdude config:', AVRDUDE_CONF
        self.AVRDUDE_BIN = AVRDUDE_BIN
        self.AVRDUDE_CONF = AVRDUDE_CONF

        if platform.system() == 'Darwin':
            # For MacOS X, pick up the AVR tools from within Arduino.app
            ARDUINO_HOME_DEFAULT = '/Applications/Arduino.app/Contents/Resources/Java'
            ARDUINO_HOME = scons_arguments.get('ARDUINO_HOME', ARDUINO_HOME_DEFAULT)
            ARDUINO_PORT_DEFAULT = getUsbTty('/dev/tty.usbserial*')
        elif platform.system() == 'Windows':
            # For Windows, use environment variables.
            #ARDUINO_PORT_DEFAULT = os.environ.get('ARDUINO_PORT')
            ARDUINO_PORT_DEFAULT = 'COM3'
        else:
            # For Ubuntu Linux (9.10 or higher)
            ARDUINO_PORT_DEFAULT = getUsbTty('/dev/ttyUSB*')
            AVR_BIN_PREFIX = 'avr-'

        ARDUINO_BOARD_DEFAULT = os.environ.get('ARDUINO_BOARD', 'mega2560')

        ARDUINO_PORT = scons_arguments.get('ARDUINO_PORT', ARDUINO_PORT_DEFAULT)
        self.ARDUINO_PORT = ARDUINO_PORT
        ARDUINO_BOARD = scons_arguments.get('ARDUINO_BOARD', ARDUINO_BOARD_DEFAULT)
        self.ARDUINO_BOARD = ARDUINO_BOARD
        ARDUINO_VER = scons_arguments.get('ARDUINO_VER', 22) # Arduino 0022
        #RST_TRIGGER = scons_arguments.get('RST_TRIGGER', 'stty hupcl -F ') # use built-in pulseDTR() by default
        RST_TRIGGER = scons_arguments.get('RST_TRIGGER', None) # use built-in pulseDTR() by default
        EXTRA_LIB = scons_arguments.get('EXTRA_LIB', None) # handy for adding another arduino-lib dir

        print 'ARDUINO_PORT:', ARDUINO_PORT

        if not ARDUINO_HOME:
            print 'ARDUINO_HOME must be defined.'
            raise KeyError('ARDUINO_HOME')

        ARDUINO_CORE = path(ARDUINO_HOME).joinpath('hardware', 'arduino',
                'cores', 'arduino')
        self.ARDUINO_VER = ARDUINO_VER
        self.ARDUINO_CORE = ARDUINO_CORE
        ARDUINO_SKEL = path(ARDUINO_CORE).joinpath('main.cpp')
        self.ARDUINO_SKEL = ARDUINO_SKEL
        ARDUINO_CONF = path(ARDUINO_HOME).joinpath('hardware', 'arduino',
                'boards.txt')

        # Some OSs need bundle with IDE tool-chain
        if platform.system() == 'Darwin' or platform.system() == 'Windows':
            AVR_BIN_PREFIX = path(ARDUINO_HOME).joinpath('hardware', 'tools',
                    'avr', 'bin', 'avr-')

        ARDUINO_LIBS = []
        if EXTRA_LIB != None:
            ARDUINO_LIBS += [EXTRA_LIB]
        ARDUINO_LIBS += [path(ARDUINO_HOME).joinpath('libraries'), ]
        self.ARDUINO_LIBS = ARDUINO_LIBS

        # check given board name, ARDUINO_BOARD is valid one
        cre_board_name = re.compile(r'^(.*)\.name=(.*)')
        boards = {}

        ARDUINO_CONFIG = path(ARDUINO_CONF).lines()
        for line in ARDUINO_CONFIG:
            result = cre_board_name.findall(line)
            if result:
                boards[result[0][0]] = result[0][1]
        self.ARDUINO_CONFIG = ARDUINO_CONFIG

        if not ARDUINO_BOARD in boards.keys():
            print ("ERROR! the given board name, %s is not in the supported board list:" % ARDUINO_BOARD)
            print ("all available board names are:")
            for name in sorted(boards.keys()):
                print ("\t%s for %s"%(name.ljust(14), boards[name]))
            print ("however, you may edit %s to add a new board." % ARDUINO_CONF)
            sys.exit(-1)

        MCU = self.get_board_conf(r'^%s\.build\.mcu=(.*)' % ARDUINO_BOARD)
        F_CPU = self.get_board_conf(r'^%s\.build\.f_cpu=(.*)' % ARDUINO_BOARD)
        self.MCU = MCU
        self.F_CPU = F_CPU

        # Verify that there is a file with the same name as the folder and with
        # the extension .pde
        TARGET = path(os.getcwd()).name
        print os.getcwd()
        pde_path = path(TARGET + '.pde')
        try:
            assert(os.path.exists(pde_path))
        except AssertionError:
            pde_files = path('.').files('*.pde')
            if not len(pde_files) == 1:
                # If there is not exactly one 'pde' file, fail.
                raise
            TARGET = pde_files[0].namebase
            pde_path = path(TARGET + '.pde')
        self.TARGET = TARGET
        self.pde_path = pde_path

        self.AVR_BIN_PREFIX = AVR_BIN_PREFIX
        if build_root is None:
            self.build_root = path('build')
        else:
            self.build_root = path(build_root)
        self.core_root = self.build_root.joinpath('core')

    def get_board_conf(self, pattern_string):
        cre_board_config = re.compile(pattern_string)
        for line in self.ARDUINO_CONFIG:
            result = cre_board_config.findall(line)
            if result:
                return result[0]
        assert(False)

    def get_env(self, **kwargs):
        cFlags = ['-ffunction-sections', '-fdata-sections', '-fno-exceptions',
                '-funsigned-char', '-funsigned-bitfields', '-fpack-struct',
                        '-fshort-enums', '-Os', '-mmcu=%s' % self.MCU]
        env_defaults = dict(CC='"%s"' % (self.AVR_BIN_PREFIX + 'gcc'),
                CXX='"%s"' % (self.AVR_BIN_PREFIX + 'g++'),
                CPPPATH=[self.core_root], CPPDEFINES={'F_CPU': self.F_CPU,
                        'ARDUINO': self.ARDUINO_VER, 'AVR': None, },
                CFLAGS=cFlags + ['-std=gnu99'], CCFLAGS=cFlags,
                TOOLS=['gcc','g++'])
        for k, v in kwargs.iteritems():
            print 'processing kwarg: %s->%s' % (k, v)
            if k in env_defaults and isinstance(env_defaults[k], dict)\
                    and isinstance(v, dict):
                env_defaults[k].update(v)
                print '  update dict'
            elif k in env_defaults and isinstance(env_defaults[k], list):
                env_defaults[k].append(v)
                print '  append to list'
            else:
                env_defaults[k] = v
                print '  set value'
        print 'kwargs:', kwargs
        print 'env_defaults:', env_defaults
        envArduino = Environment(**env_defaults)
        # Add Arduino Processing, Elf, and Hex builders to environment
        for builder_name in ['Processing', 'Elf', 'Hex']:
            envArduino.Append(BUILDERS={builder_name: getattr(self,
                    'get_%s_builder' % builder_name.lower())()})
        return envArduino

    def get_processing_builder(self):
        def fnProcessing(target, source, env):
            wp = open('%s' % target[0], 'wb')
            wp.write(open(self.ARDUINO_SKEL).read())
            # Add this preprocessor directive to localize the errors.
            sourcePath = str(source[0]).replace('\\', '\\\\');
            wp.write('#line 1 "%s"\r\n' % sourcePath)
            wp.write(open('%s' % source[0]).read())
            wp.close()
            return None
        return Builder(action=fnProcessing, suffix='.cpp', src_suffix='.pde')

    def get_elf_builder(self):
        return Builder(action='"%s"' % (self.AVR_BIN_PREFIX + 'gcc') + \
                ' -mmcu=%s -Os -Wl,--gc-sections -o $TARGET $SOURCES -lm' % (
                        self.MCU))

    def get_hex_builder(self):
        return Builder(action='"%s"' % (self.AVR_BIN_PREFIX + 'objcopy') +\
                ' -O ihex -R .eeprom $SOURCES $TARGET')

    def get_avrdude_options(self):
        UPLOAD_PROTOCOL = self.get_board_conf(r'^%s\.upload\.protocol=(.*)' % (
                self.ARDUINO_BOARD))
        UPLOAD_SPEED = self.get_board_conf(r'^%s\.upload\.speed=(.*)' % (
                self.ARDUINO_BOARD))

        avrdudeOpts = ['-V', '-F', '-c %s' % UPLOAD_PROTOCOL, '-b %s' % UPLOAD_SPEED,
            '-p %s' % self.MCU, '-P %s' % self.ARDUINO_PORT, '-U flash:w:$SOURCES']

        if self.AVRDUDE_CONF:
            avrdudeOpts += ['-C "%s"' % self.AVRDUDE_CONF]
        return avrdudeOpts

    def get_core_sources(self):
        # Generate list of Arduino core source files
        core_sources = gather_sources(self.ARDUINO_CORE)
        core_sources = [x for x in core_sources
                if not (x.name == 'main.cpp')]
        core_sources = [x.replace(self.ARDUINO_CORE, self.core_root)
                for x in core_sources]
        return core_sources

    def get_lib_candidate_list(self, arduino_version):
        return get_lib_candidate_list(self.pde_path, arduino_version)

    def get_lib_sources(self, lib_candidates, arduino_libs, env):
        '''
        Add VariantDir references for all libraries in lib_candidates to the
        corresponding paths in arduino_libs.
        
        Return the combined list of source files for all libraries, relative
        to the respective VariantDir.
        '''
        all_libs_sources = []
        all_lib_names = set()
        index = 0
        for orig_lib_dir in arduino_libs:
            lib_sources = []
            lib_dir = self.build_root.joinpath('lib_%02d' % index)
            print 'build_root: %s' % self.build_root
            env.VariantDir(lib_dir, orig_lib_dir)
            for libPath in path(orig_lib_dir).dirs():
                libName = libPath.name
                if not libName in lib_candidates:
                    # This library is not included in the .pde file, so skip it
                    continue
                elif libName in all_lib_names:
                    # This library has already been processed, so skip it
                    continue
                all_lib_names.add(libName)
                env.Append(CPPPATH=libPath.replace(orig_lib_dir, lib_dir))
                lib_sources = gather_sources(libPath)
                utilDir = path(libPath).joinpath('utility')
                if os.path.exists(utilDir) and os.path.isdir(utilDir):
                    lib_sources += gather_sources(utilDir)
                    env.Append(CPPPATH=utilDir.replace(orig_lib_dir, lib_dir))
                lib_sources = [x.replace(orig_lib_dir, lib_dir) for x in lib_sources]
                all_libs_sources += lib_sources
            index += 1
        return all_libs_sources

    def build(self, hex_root=None, env_dict=None, extra_sources=None, register_upload=False):
        if hex_root is None:
            hex_root = self.build_root.joinpath('hex')
        else:
            hex_root = path(hex_root)
            if not hex_root.isabs():
                hex_root = self.build_root.joinpath(hex_root)
        hex_path = hex_root.joinpath(self.TARGET + '.hex')

        if env_dict is None:
            env_dict = {}
        env = self.get_env(**env_dict)
        print env['CPPDEFINES']

        # Convert sketch(.pde) to cpp
        env.Processing(hex_root.joinpath(self.TARGET+'.cpp'), hex_root.joinpath(
                self.pde_path))

        sources = [hex_root.joinpath(self.TARGET+'.cpp')]
        sources += self.get_lib_sources(self.get_lib_candidate_list(
                self.ARDUINO_VER), self.ARDUINO_LIBS, env)
        sources += self.get_core_sources()
        if extra_sources:
            sources += [hex_root.joinpath(s) for s in extra_sources]

        # Finally Build!!
        objs = env.Object(sources) #, LIBS=libs, LIBPATH='.')
        #objs_1 = env.Object(sources, CPPDEFINES={'___HARDWARE_MINOR_VERSION___': 1})
        elf_path = hex_root.joinpath(self.TARGET + '.elf')
        env.Elf(elf_path, objs)
        arduino_hex = env.Hex(hex_path, hex_root.joinpath(self.TARGET + '.elf'))

        # Print Size
        # TODO: check binary size
        MAX_SIZE = self.get_board_conf(
                r'^%s\.upload.maximum_size=(.*)' % self.ARDUINO_BOARD)
        print ("maximum size for hex file: %s bytes" % MAX_SIZE)
        env.Command(None, hex_path, '"%s"' % (self.AVR_BIN_PREFIX + 'size'
                ) + ' --target=ihex $SOURCE')

        if register_upload:
            fuse_cmd = '"%s" %s' % (self.AVRDUDE_BIN, ' '.join(
                    self.get_avrdude_options()))

            upload = env.Alias('upload', hex_path, [fuse_cmd]);
            env.AlwaysBuild(upload)

        # Clean build directory
        env.Clean('all', 'build/')

        env.VariantDir(self.core_root, self.ARDUINO_CORE)
        env.VariantDir(hex_root, '.')
        return arduino_hex
