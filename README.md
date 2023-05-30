[![github url](https://img.shields.io/badge/github-url-000.svg?logo=github&labelColor=gray&color=blue)](https://github.com/nlesc-recruit/PowerSensor3)
[![License](https://img.shields.io/github/license/nlesc-recruit/PowerSensor3)](https://github.com/nlesc-recruit/PowerSensor3)
[![DOI](https://zenodo.org/badge/455610726.svg)](https://zenodo.org/badge/latestdoi/455610726)
[![Research Software Directory](https://img.shields.io/badge/RSD-PowerSensor3-00a3e3.svg)](https://www.research-software-directory.org/software/powersensor3)
[![CII Best Practices](https://bestpractices.coreinfrastructure.org/projects/7401/badge)](https://bestpractices.coreinfrastructure.org/projects/7401)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/77f179fdc0c84de3aa5420a99bddf84a)](https://www.codacy.com/gh/nlesc-recruit/PowerSensor3/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=nlesc-recruit/PowerSensor3&amp;utm_campaign=Badge_Grade)
[![Device workflow](https://img.shields.io/github/actions/workflow/status/nlesc-recruit/PowerSensor3/build_device.yml?label=build%20device%20firmware)](https://github.com/nlesc-recruit/PowerSensor3/actions/workflows/build_device.yml)
[![Host workflow](https://img.shields.io/github/actions/workflow/status/nlesc-recruit/PowerSensor3/build_host.yml?label=build%20host%20library)](https://github.com/nlesc-recruit/PowerSensor3/actions/workflows/build_host.yml)
[![Citation metadata](https://github.com/nlesc-recruit/PowerSensor3/actions/workflows/cffconvert.yml/badge.svg)](https://github.com/nlesc-recruit/PowerSensor3/actions/workflows/cffconvert.yml)

## Introduction
PowerSensor3 is a tool that measures the instantaneous power consumption of PCIe cards and SoC development boards like GPUs, Xeon Phis, FPGAs, DSPs, and network cards, at sub-millisecond time scale. It consists of a commodity microcontroller, commodity current and voltage sensors, and (for PCIe devices) a PCIe riser card. The microcontroller reports measurements to the host via USB. A small host library allows an application to determine its own energy efficiency. The high time resolution provides much better insight into energy usage than low-resolution built-in power meters (if available at all), as PowerSensor enables analysis of individual compute kernels.

## Hardware
PowerSensor3 uses a modular hardware design with a base PCB containing a microcontroller and optionally a display, and up to four plug-in sensor boards. The PCB design, developed using KiCAD, is openly available [here](https://git.astron.nl/RD/powersensor3).

The default configuration of PowerSensor3 is an STM32F401 Black Pill microcontroller with SPI-based display, and up to four pairs of voltage and current sensors. Typical usage with a GPU is to measure both the 3.3 V and 12 V power through the PCIe slot, and up to two ATX 12 V connections directly from the power supply to the GPU. The microcontroller interprets the sensor data and reports the power measurements via USB to the host.

## Disclaimer
You are about to modify electronic circuits.  Improperly connecting wires can be dangerous and may damage computer equipment.  We are not responsible for any harm or damage that might occur.  Make sure that you understand very well how all cables should be connected.  Do not install this tool if you are not absolutely sure what you are doing.

## Getting started
1. Clone this repository including submodules: `git clone https://github.com/nlesc-recruit/PowerSensor3 --recurse-submodules`. If you have alread cloned the repository without submodules, run the following in the root of the repository: `git submodule init; git submodule update`.
2. [Installation of the physical device and firwmare](docs/INSTALLATION_DEVICE.md)
3. [Installation of the host library](docs/INSTALLATION_HOST.md)
3. [PowerSensor user guide](docs/USERGUIDE.md)

## Further reading
Altering the firmware of the device could be necessary in certain use cases. For this please refer to the following documents:

* [STM32F401CCU6 BlackPill overview](https://stm32-base.org/boards/STM32F401CCU6-WeAct-Black-Pill-V1.2.html)
* [STM32F401xB/C Datasheet](https://www.st.com/resource/en/datasheet/stm32f401cc.pdf)
* [STM32F401xB/C/D/E Reference manual](https://www.st.com/resource/en/reference_manual/rm0368-stm32f401xbc-and-stm32f401xde-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
* [STM32F2/4/7 DMA programming manual](https://www.st.com/resource/en/application_note/dm00046011-using-the-stm32f2-stm32f4-and-stm32f7-series-dma-controller-stmicroelectronics.pdf)
* [STM32F40x/STM3241x EEPROM emulation in flash](https://www.st.com/resource/en/application_note/an3969-eeprom-emulation-in-stm32f40xstm32f41x-microcontrollers-stmicroelectronics.pdf)
