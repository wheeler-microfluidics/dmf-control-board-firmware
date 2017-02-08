@echo off
set PKG_NAME=dmf-control-board-firmware
setlocal enableextensions
md "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v1_1
md "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v1_2
md "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v1_3
md "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v2_0
md "%PREFIX%"\Library\bin\platformio\%PKG_NAME%\mega2560_hw_v2_1
endlocal

REM Generate Arduino/Python code
paver create_config

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
"%PYTHON%" -m paver generate_setup
if errorlevel 1 exit 1

REM **Workaround** `conda build` runs a copy of `setup.py` named
REM `conda-build-script.py` with the recipe directory as the only argument.
REM This causes paver to fail, since the recipe directory is not a valid paver
REM task name.
REM
REM We can work around this by wrapping the original contents of `setup.py` in
REM an `if` block to only execute during package installation.
"%PYTHON%" -c "from __future__ import print_function; input_ = open('setup.py', 'r'); data = input_.read(); input_.close(); output_ = open('setup.py', 'w'); output_.write('\n'.join(['import sys', 'import path_helpers as ph', '''if ph.path(sys.argv[0]).name == 'conda-build-script.py':''', '    sys.argv.pop()', 'else:', '\n'.join([('    ' + d) for d in data.splitlines()])])); output_.close(); print(open('setup.py', 'r').read())"
if errorlevel 1 exit 1

REM Build Python C-extension
echo "Build Python C-extension"
REM **Workaround** Run `scons` in subprocess, since otherwise it causes the
REM build script to exit early, preventing installation of the Python package
REM (not sure why this happens...).
"%PYTHON%" -c "import subprocess as sp; sp.check_call('scons', shell=True)"
if errorlevel 1 exit 1

REM Install source directory as Python package.
echo "Install source directory as Python package."
"%PYTHON%" setup.py install --single-version-externally-managed --record record.txt
if errorlevel 1 exit 1
