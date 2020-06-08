[![CII Best Practices](https://bestpractices.coreinfrastructure.org/projects/3701/badge)](https://bestpractices.coreinfrastructure.org/projects/3701)
## Introduction
PowerSensor is a tool that measures the instantaneous power consumption of PCIe cards and SoC development boards like GPUs, Xeon Phis, FPGAs, DSPs, and network cards, at sub-millisecond time scale. It consists of a commodity microcontroller, commodity current sensors, and (for PCIe devices) a PCIe riser card. The microcontroller reports measurements to the host via USB. A small host library allows an application to determine its own energy efficiency. The high time resolution provides much better insight into energy usage than low-resolution built-in power meters (if available at all), as PowerSensor enables analysis of individual compute kernels.

## Hardware
A PowerSensor measures the instantaneous power use of a GPU:

![The PowerSensor hardware](https://i.imgur.com/zEu4LSS.jpg)

The PowerSensor consists of an STM32F407VG board, current sensors (ACS712), a PCIe riser cable (to measure the power drawn from the motherboard) and a USB cable that connects to the host. In this scenario, we use three sensors that measure the PCIe slot power (12 V and 3.3 V) and the external PCIe cable power. The microcontroller interprets the sensor data and reports the power measurements via USB to the host. The STM32 board can also be swapped with the Arduino Leonardo.

![The PowerSensor schematics](https://i.imgur.com/6C1UhWO.png)

## Example
PowerSensor gives insight into an application’s power efficiency:

![Continuous power measurements of several devices](https://i.imgur.com/luMUGgM.png)

This radio-astronomical pipeline filters, corrects, and correlates the signals from 960 receivers. The figure shows the instantaneous power consumption of three different devices. The correlation between energy use and executed kernels is clearly visible. The shaded area below the curve corresponds to the total energy used by a kernel.

## Disclaimer
You are about to modify electronic circuits.  Improperly connecting wires can be dangerous and may damage computer equipment.  We are not responsible for any harm or damage that might occur.  Make sure that you understand very well how all cables should be connected.  Do not install this tool if you are not absolutely sure what you are doing.

## Getting started
1. [Installation the PowerSensor]{docs/INSTALLATION.md}
2. Using the PowerSensor




