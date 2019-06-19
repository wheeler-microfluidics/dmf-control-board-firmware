@echo off
set PKG_NAME=dmf-control-board-firmware
set MODULE_NAME=dmf_control_board_firmware
setlocal enableextensions
md "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v1_1
md "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v1_2
md "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v1_3
md "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v2_0
md "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v2_1
md "%SP_DIR%"\dmf_control_board_firmware
endlocal

REM Copy mingw linked library to package directory.
copy "%PREFIX%"\Library\tdm-mingw\bin\mingwm10.dll "%SP_DIR%"\dmf_control_board_firmware\mingwm10.dll
if errorlevel 1 exit 1

REM Generate Arduino/Python code
python -m paver create_config

REM REM Build firmware
pio run
if errorlevel 1 exit 1
REM REM Copy compiled firmware to Conda bin directory
copy "%SRC_DIR%"\platformio.ini "%PREFIX%"\Library\bin\platformio\%PKG_NAME%
copy "%SRC_DIR%"\.pioenvs\mega2560_hw_v1_1\firmware.hex "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v1_1\firmware.hex
copy "%SRC_DIR%"\.pioenvs\mega2560_hw_v1_2\firmware.hex "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v1_2\firmware.hex
copy "%SRC_DIR%"\.pioenvs\mega2560_hw_v1_3\firmware.hex "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v1_3\firmware.hex
copy "%SRC_DIR%"\.pioenvs\mega2560_hw_v2_0\firmware.hex "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v2_0\firmware.hex
copy "%SRC_DIR%"\.pioenvs\mega2560_hw_v2_1\firmware.hex "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v2_1\firmware.hex
if errorlevel 1 exit 1

REM Generate `setup.py` from `pavement.py` definition.
python -m paver generate_setup
if errorlevel 1 exit 1

REM XXX **Workaround** `conda build` runs a copy of `setup.py` named
REM `conda-build-script.py` with the recipe directory as the only argument.
REM This causes paver to fail, since the recipe directory is not a valid paver
REM task name.
REM
REM We work around this by wrapping the original contents of `setup.py` in an
REM `if` block to only execute during package installation.
python -c "from __future__ import print_function; input_ = open('setup.py', 'r'); data = input_.read(); input_.close(); output_ = open('setup.py', 'w'); output_.write('\n'.join(['import sys', 'import path_helpers as ph', '''if ph.path(sys.argv[0]).name == 'conda-build-script.py':''', '    sys.argv.pop()', 'else:', '\n'.join([('    ' + d) for d in data.splitlines()])])); output_.close(); print(open('setup.py', 'r').read())"
if errorlevel 1 exit 1

REM Build Python C-extension
echo "Build Python C-extension"
REM XXX **Workaround** Run `scons` in subprocess, since otherwise it causes the
REM build script to exit early, preventing installation of the Python package
REM (not sure why this happens...).
python -c "import subprocess as sp; sp.check_call('scons', shell=True)"
if errorlevel 1 exit 1

REM Install source directory as Python package.
echo "Install source directory as Python package."
python setup.py install --single-version-externally-managed --record record.txt
if errorlevel 1 exit 1
