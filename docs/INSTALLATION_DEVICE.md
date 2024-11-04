# Device installation guide
Start with the physical installation of the PowerSensor, then install the firmware to the device. When both are completed succesfully, install the host library with [this guide](INSTALLATION_HOST.md) and get to know the device with the [user guide](USERGUIDE.md).

## Installing the PowerSensor
Before starting, turn off and unplug the power of the host system. In the case of a GPU, a PCI-e riser card or cable is needed to be able to measure the 3.3 V and 12 V power coming from the PCI-e slot of the motherboard. 12 V power coming directly from the PSU can be connected to a PowerSensor3 sensor board. A second ATX power cable should be used to connect the PowerSensor3 sensor board to the GPU. Take care to use sensor boards with appropiate voltage and current sensors, taking into account the maximum voltage and current:

Connection  | Voltage (V) | Maximum power (W) | Maximum current (A)
------------|-------------|-------------------|--------------------
PCI-e 3.3 V | 3.3         | 10                | 3.0
PCI-e 12 V  | 12          | 65                | 5.4
ATX 6-pin   | 12          | 75                | 6.3
ATX 8-pin   | 12          | 150               | 12.5

After connecting all relevant power cables, connect the microcontroller to a USB port on the host. Make sure that everything is connected correctly, then turn on the host system.

## Uploading the firmware
Pre-built firmware can be downloaded from the [PowerSensor3 Github page](https://github.com/nlesc-recruit/PowerSensor3/releases). Alternatively, see the next section for building the firwmare from source.  
Before uploading the firwmare, ensure the device is booted in DFU mode. This is typically achieved by holding down the BOOT0 button and pressing the RESET button. Confirm that the device has entered DFU mode with one of the following commands:

    lsusb
    dfu-util -l

The firmware can be uploaded with `dfu-util` or `arduino-cli`. The latter can only be used when compiling the firmware yourself.
To upload with `dfu-util` run the following command, adapting the -a and -i options as necessary to match the output of `dfu-util -l`:

    dfu-util -a 0 -i 0 -s 0x08000000:leave -D /path/to/PowerSensor3/firmware.bin

To upload with `arduino-cli`, first make sure you can build the firmware as outlined in the next section.
Then run the following command in the device folder, adding any flags in the same way as with the `make device` command.

    make upload

## Building the firmware
We provide pre-built binaries [here](https://github.com/nlesc-recruit/PowerSensor3/releases) for default configurations using either any of the supported microcontrollers. For non-default settings or other customizations, the firmware can be built with the Arduino toolkit as outlined in this section. Note that the pre-built binaries use a modified USB transmit buffer size, see the [USB Buffer size](#usb-buffer-size) section.


The firmware is dependent on a few Arduino tools, these should be installed before continueing. First the [arduino-cli](https://github.com/arduino/arduino-cli) package can be installed on Linux via:

    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

Next, the [stm32duino](https://github.com/stm32duino/Arduino_Core_STM32) package should be installed, this ensures that STM32 boards can be used in combination with arduino. Start by updating the arduino core:

    arduino-cli core update-index

Then, if there is not already a config file at `~/.arduino15/arduino-cli.yaml`, create a config file with:

    arduino-cli config init

Open the configuration file in any editor and add new board manager URL to:

    board_manager:
      additional_urls:
        [https://github.com/stm32duino/BoardManagerFiles/raw/master/STM32/package_stm_index.json]

Update the core again:

    arduino-cli core update-index

arduino-cli will automatically install the STM32 core when building the firmware.  
**NOTE**, stm32duino version 2.5.0 and later do not work with PowerSensor3. See [this Github issue](https://github.com/nlesc-recruit/PowerSensor3/issues/125) for details and a workaround.

Then run the following command in the device folder to build the firmware with default flags:

    make device

For an STM32F401 or STM32F411 Black Pill, the fimware will be written to `PowerSensor/build/STMicroelectronics.stm32.GenF4/PowerSensor.ino.bin`, for the STM32F407 Discovery, the file location is `PowerSensor/build/STMicroelectronics.stm32.Disco/PowerSensor.ino.bin`.

If the firmware is uploaded successfully, the device will be reset and start running the PowerSensor3 firmware.

### Build customization
There are several options available to customize the firmware build. These options can be append to the `make device` and `make upload` commands.

### Target microcontroller
A flag is provided to set whether the firmware is built for an STM32F401, STM32F411 (default) or STM32F407 microcontroller.  
Option name: DEV  
Allowed values: F401, F411, F407  
Example:

    make upload DEV="F401"


### Extra flags
Defines that would usually be given to the compiler with the `-D` option can be set with `make` using the `FLAGS` option.
Currently supported flags relate to the display:
`-DNODISPLAY` disables the display completely. This also means that the external libraries, located in `PowerSensor/Libraries` are not used.
`-DTFT_BLUE` changes the display type from the default green tab to a blue tab. Effectively, this only inverts the display colours.
`-DDEMO` enables updating the display during measurements, meant for demos. Updating the display interferes with the measurement interval, so this mode should not be used when accurate measurements are required.
Multiple flags should be separated by a space.
Example command:

    make upload FLAGS="-DTFT_BLUE"

### USB buffer size
The STM32 USB library has a default transmit buffer size that is too small to handle the high data rate used in PowerSensor3. When using the default buffer size, you will most likely see dropped data. The buffer size is set in the `cdc_queue.h` file, part of stm32duino. This file is usually located at `<Arduino folder>/packages/STMicroelectronics/hardware/stm32/2.3.0/cores/arduino/stm32/usb/cdc/cdc_queue.h`

We provide a patch file to increase the buffer size (tested with version 2.3.0 of stm32duino), as well as a Python script to locate the stm32duino folder. To automatically update the `cdc_queue.h` file, run something like this from the root of the PowerSensor3 repository:

    REPO_ROOT=$PWD
    STM32_DIR=$(python/get_arduino_stm32_directory.py)
    cd ${STM32_DIR}/cores/arduino/stm32/usb/cdc
    patch < ${REPO_ROOT}/patch/cdc_queue.patch

Then proceed with building the firmware with `arduino-cli` as usual.

## Next steps
After installing the device, proceed with installing the host library with [this guide](INSTALLATION_HOST.md).
