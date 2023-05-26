[![Codacy Badge](https://app.codacy.com/project/badge/Grade/77f179fdc0c84de3aa5420a99bddf84a)](https://www.codacy.com/gh/nlesc-recruit/PowerSensor3/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=nlesc-recruit/PowerSensor3&amp;utm_campaign=Badge_Grade)
[![CII Best Practices](https://bestpractices.coreinfrastructure.org/projects/3701/badge)](https://bestpractices.coreinfrastructure.org/projects/3701)
[![Research Software Directory](https://img.shields.io/badge/RSD-PowerSensor3-00a3e3.svg)](https://www.research-software-directory.org/software/powersensor3)

## Introduction
PowerSensor is a tool that measures the instantaneous power consumption of PCIe cards and SoC development boards like GPUs, Xeon Phis, FPGAs, DSPs, and network cards, at sub-millisecond time scale. It consists of a commodity microcontroller, commodity current and voltage sensors, and (for PCIe devices) a PCIe riser card. The microcontroller reports measurements to the host via USB. A small host library allows an application to determine its own energy efficiency. The high time resolution provides much better insight into energy usage than low-resolution built-in power meters (if available at all), as PowerSensor enables analysis of individual compute kernels.

## Hardware
A PowerSensor measures the instantaneous power use of a GPU:

<!-- ![The PowerSensor hardware](https://i.imgur.com/zEu4LSS.jpg) -->

The PowerSensor consists of an STM32F401CCU6 BlackPill board, current and voltage sensors, a PCIe riser cable (to measure the power drawn from the motherboard) and a USB cable that connects to the host. Up to four pairs of voltage/current sensors can be connected to PowerSensor. Typical usage with a GPU is to measure both the 3.3 V and 12 V power through the PCIe slot, and up to two ATX 12 V connections directly from the power supply to the GPU. The microcontroller interprets the sensor data and reports the power measurements via USB to the host.

<!-- ![The PowerSensor schematics](https://i.imgur.com/6C1UhWO.png) -->

## Example
PowerSensor gives insight into an application’s power efficiency:

![Continuous power measurements of several devices](https://i.imgur.com/luMUGgM.png)

This radio-astronomical pipeline filters, corrects, and correlates the signals from 960 receivers. The figure shows the instantaneous power consumption of three different devices. The correlation between energy use and executed kernels is clearly visible. The shaded area below the curve corresponds to the total energy used by a kernel.

## Disclaimer
You are about to modify electronic circuits.  Improperly connecting wires can be dangerous and may damage computer equipment.  We are not responsible for any harm or damage that might occur.  Make sure that you understand very well how all cables should be connected.  Do not install this tool if you are not absolutely sure what you are doing.

## Getting started
1. Clone this repository including submodules: `git clone https://github.com/nlesc-recruit/PowerSensor3 --recurse-submodules`. If you have alread cloned the repository without submodules, run the following in the root of the repository: `git submodule init; git submodule update`.
2. [Installing the PowerSensor](docs/INSTALLATION.md)
3. [Using the PowerSensor](docs/USERGUIDE.md)

## Further reading
Altering the software of the device could be necessary in certain use cases. For this please refer to the following documents:

* [STM32F401CCU6 BlackPill overview](https://stm32-base.org/boards/STM32F401CCU6-WeAct-Black-Pill-V1.2.html)
* [STM32F401xB/C Datasheet](https://www.st.com/resource/en/datasheet/stm32f401cc.pdf)
* [STM32F401xB/C/D/E Reference manual](https://www.st.com/resource/en/reference_manual/rm0368-stm32f401xbc-and-stm32f401xde-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
* [STM32F2/4/7 DMA programming manual](https://www.st.com/resource/en/application_note/dm00046011-using-the-stm32f2-stm32f4-and-stm32f7-series-dma-controller-stmicroelectronics.pdf)
* [STM32F40x/STM3241x EEPROM emulation in flash](https://www.st.com/resource/en/application_note/an3969-eeprom-emulation-in-stm32f40xstm32f41x-microcontrollers-stmicroelectronics.pdf)
