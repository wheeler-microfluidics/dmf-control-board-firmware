mkdir -p "${PREFIX}"/include/Arduino
mkdir -p "${PREFIX}"/bin/platformio/dropbot-dx/teensy31

# Generate Arduino code
"${PYTHON}" -m paver generate_all_code
rc=$?; if [[ $rc != 0  ]]; then exit $rc; fi

# Build firmware
"${PYTHON}" -m paver build_firmware
rc=$?; if [[ $rc != 0  ]]; then exit $rc; fi

# Copy Arduino library to Conda include directory
cp -ra "${SRC_DIR}"/lib/DropbotDx "${PREFIX}"/include/Arduino/DropbotDx
# Copy compiled firmware to Conda bin directory
cp -a "${SRC_DIR}"/lib/.pioenvs/teensy31/firmware.hex "${PREFIX}"/bin/dropbot-dx/teensy31/firmware.hex
rc=$?; if [[ $rc != 0  ]]; then exit $rc; fi

# Generate `setup.py` from `pavement.py` definition.
"${PYTHON}" -m paver generate_setup
rc=$?; if [[ $rc != 0  ]]; then exit $rc; fi

# **Workaround** `conda build` runs a copy of `setup.py` named
# `conda-build-script.py` with the recipe directory as the only argument.
# This causes paver to fail, since the recipe directory is not a valid paver
# task name.
#
# We can work around this by wrapping the original contents of `setup.py` in
# an `if` block to only execute during package installation.
"${PYTHON}" -c "from __future__ import print_function; input_ = open('setup.py', 'r'); data = input_.read(); input_.close(); output_ = open('setup.py', 'w'); output_.write('\n'.join(['import sys', 'import path_helpers as ph', '''if ph.path(sys.argv[0]).name == 'conda-build-script.py':''', '    sys.argv.pop()', 'else:', '\n'.join([('    ' + d) for d in data.splitlines()])])); output_.close(); print(open('setup.py', 'r').read())"

# Install source directory as Python package.
"${PYTHON}" -m pip install --no-cache .
rc=$?; if [[ $rc != 0  ]]; then exit $rc; fi
