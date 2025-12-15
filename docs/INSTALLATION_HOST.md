# Host library installation guide
After installing the physical device and firmware (see [this guide](INSTALLATION_DEVICE.md)), use this guide to install the host library.

## Installation
The PowerSensor3 host library can be built with cmake. Make sure you have a C++ compiler installed and run the following commands in the `host` directory to build and install to the default location (typically `/usr/local/`):

    cmake -S . -B build
    cd build
    make
    make install

To install to a different location, use the `-DCMAKE_INSTALL_PREFIX` option of cmake.

## Python bindings
Python bindings can be generated with Pybind11. To use these, make sure you have a recent version of Python3 and install Pybind11, typically with

    pip install pybind11

To compile the PowerSensor3 Python bindings, add `-DPYTHON_BINDINGS=ON` to the cmake command. You may need to point cmake to the Pybind11 directory with `-Dpybind11_DIR=$(pybind11-config --cmakedir)`. This generates a shared library under `build/python`, which you can copy to any folder where Python can find it, or you can add the folder containing the library to `PYTHONPATH`.

## Next steps
To test your installation calibrate the device, have a look at the [configuration guide](CONFIGURATION.md).
