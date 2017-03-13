dmf-control-board-firmware
==========================

Firmware for an [Arduino-based DMF control board][1] and a Python module for
communicating with it over a serial connection.

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [Install](#install)
- [Conda package contents](#conda-package-contents)
- [Build Conda package (including firmware and host driver)](#build-conda-package-including-firmware-and-host-driver)
- [Develop](#develop)
  - [Set up development environment (within a Conda environment)](#set-up-development-environment-within-a-conda-environment)
  - [Build firmware](#build-firmware)
  - [Flash/upload firmware](#flashupload-firmware)
  - [Build host driver (Python C-extension)](#build-host-driver-python-c-extension)
  - [Unlink development working copy](#unlink-development-working-copy)
- [Contributors](#contributors)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

-------------------------------------------------------------------------------

Install
-------

The latest [`dmf-control-board-firmware` release][3] is available as a
[Conda][2] package from the [`wheeler-microfluidics`][4] channel using.

To install `dmf-control-board-firmware` in a Conda environment, run:

    conda install -c wheeler-microfluidics dmf-control-board-firmware

-------------------------------------------------------------------------------

Conda package contents
----------------------

The `dmf_control_board_firmware` Conda package includes:

 - A **low-level host driver** (written in **C++**) for communicating with the
   **control board** over a serial interface (exposed to Python through
   **[Boost Python][6] bindings**).
 - A `dmf_control_board_firmware.DMFControlBoard` **Python class** providing a
   high-level wrapper around the **[Boost Python][6] bindings** C++ API.
 - **Compiled firmware binaries** for each control board hardware revision.

The installed components (relative to the root of the Conda environment) are
shown below:

    ├───Lib
    │   └───site-packages
    │       └───dmf_control_board_firmware (Python package including host driver)
    │           │   ...
    │           │   dmf_control_board_base.pyd
    │           │   libboost_python-mgw44-mt-1_46_1.dll
    │           │   mingwm10.dll
    │           │   ...
    │
    └───Library
        └───bin
            └───platformio
                └───dmf-control-board-firmware (compiled firmware binaries)
                    │   platformio.ini   (PlatformIO environment information)
                    │
                    ├───mega2560_hw_v1_1
                    │       firmware.hex
                    │
                    ├───mega2560_hw_v1_2
                    │       firmware.hex
                    │
                    ├───mega2560_hw_v1_3
                    │       firmware.hex
                    │
                    ├───mega2560_hw_v2_0
                    │       firmware.hex
                    │
                    └───mega2560_hw_v2_1
                            firmware.hex

-------------------------------------------------------------------------------

Build Conda package (including firmware and host driver)
--------------------------------------------------------

Install `conda-build`:

    conda install conda-build

Build Conda package (including firmware binaries and host C-extension driver)
from included recipe:

    conda build .conda-recipe

**(Optional)** Install built Conda package (including control board firmware
and host C-extension driver):

    conda install -c wheeler-microfluidics --use-local dmf-control-board-firmware

-------------------------------------------------------------------------------

Develop
-------

### Set up development environment (within a Conda environment) ###

 1. **Clone `dmf-control-board-firmware`** source code from [GitHub repository][5].
 2. **Install development dependencies** (including dependencies required for
    `paver develop_link` command in **3**):

    conda install -c wheeler-microfluidics dmf-control-board-firmware-develop

 3. Run the following command within the root of the cloned repository to
    **install run-time dependencies** and link working copy of firmware
    binaries and Python package for run-time use:

        paver develop_link

### Build firmware ###

Run the following command within the root of the cloned repository to **build
the firmware** (host driver `.pyd`/`.dll` files will not be built):

    paver build_firmware

**N.B.,** the `paver build_firmware` command will build a separate firmware
binary for each hardware environment listed in the `platformio.ini` file (e.g.,
`mega2560_hw_v1_1`, `mega2560_hw_v2_1`).

The compiled firmware binaries are available under the `.pioenvs` directory, as
shown below:

    └───.pioenvs
        ├───mega2560_hw_v1_1
        │       firmware.hex
        │
        ├───mega2560_hw_v1_2
        │       firmware.hex
        │
        ├───mega2560_hw_v1_3
        │       firmware.hex
        │
        ├───mega2560_hw_v2_0
        │       firmware.hex
        │
        └───mega2560_hw_v2_1
                firmware.hex

### Flash/upload firmware ###

To flash/upload a compiled firmware to a connected Arduino Mega2560 board, run
the following command from the root of the repository:

    pio run --target upload --target nobuild -e <hardware environment> --upload-port <COM port>

replacing:

 - `<hardware environment>` with the tag corresponding to the **control board
   hardware version** (e.g., `mega2560_hw_v2_1`)
 - `<COM port>` with the serial port corresponding to the connected **control
   board** Arduino Mega2560 (e.g., `COM1`)

### Build host driver (Python C-extension) ###

The **low-level host driver** for communicating with the **control board** over a
serial interface is written in **C++** and exposing a Python API through
**[Boost Python][6] bindings**.

**N.B.**, the `dmf_control_board_firmware.DMFControlBoard` Python class
provides a high-level wrapper around the C++ class API.

Run the following command within the root of the cloned repository to **build
the [Boost Python][6]** bindings (i.e., `.pyd`/`.dll` files):

    scons

### Unlink development working copy ###

Run the following command within the root of the cloned repository to unlink
working copy of firmware binaries and Python package:

    paver develop_unlink

This will allow, for example, installation of a main-line release of the
`dmf-control-board-firmware` Conda package.

-------------------------------------------------------------------------------

Contributors
------------

 - Ryan Fobel ([@ryanfobel](https://github.com/ryanfobel))
 - Christian Fobel ([@cfobel](https://github.com/cfobel))


[1]: http://microfluidics.utoronto.ca/trac/dropbot/wiki/ControlBoard
[2]: https://github.com/conda/conda
[3]: https://anaconda.org/wheeler-microfluidics/dmf-control-board-firmware
[4]: https://anaconda.org/wheeler-microfluidics
[5]: https://github.com/wheeler-microfluidics/dmf-control-board-firmware
[6]: https://wiki.python.org/moin/boost.python/GettingStarted
