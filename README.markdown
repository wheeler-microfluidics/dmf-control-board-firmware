dmf-control-board-firmware
==========================

Firmware for an [Arduino-based DMF control board][1] and a Python module for
communicating with it over a serial connection.


Install
-------

The latest [`dmf-control-board-firmware` release][3] is available as a
[Conda][2] package from the [`wheeler-microfluidics`][4] channel using.

To install `dmf-control-board-firmware` in a Conda environment, run:

    conda install -c wheeler-microfluidics dmf-control-board-firmware


Build
-----

Install `conda-build`:

    conda install conda-build

Build recipe:

    conda build .


Contributors
------------

 - Ryan Fobel ([@ryanfobel](https://github.com/ryanfobel))
 - Christian Fobel ([@cfobel](https://github.com/cfobel))


[1]: http://microfluidics.utoronto.ca/trac/dropbot/wiki/ControlBoard
[2]: https://github.com/conda/conda
[3]: https://anaconda.org/wheeler-microfluidics/dmf-control-board-firmware
[4]: https://anaconda.org/wheeler-microfluidics
