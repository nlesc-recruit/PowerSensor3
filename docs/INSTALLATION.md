# Installation guide
## Installing the PowerSensor
Before starting, turn off and unplug the power of the host system.  An STM32F407VG is at the heart of the PowerSensor.  The microcontroller on this board collects the sensor values, interprets them, and communicates them to the host via USB. 

PowerSensor measures currents, using external current sensors (like those from the ACS712 series).  The current sensor, in this case a Hall-effect sensor, measures the current that flows through a wire, and converts the input current to an (analogue) voltage that is directly proportional to the current.  This voltage is sampled by the built-in ADC converter of the Arduino.  PowerSensor assumes that no current (0 Ampere) translates to a 2.5V signal.

![Imgur](https://i.imgur.com/K1WXNHY.jpg)

To measure the power of a PCIe device, a riser cable is necessary, so that the current drawn from the PCIe slot can be measured.  An appropriate rised cable is necessary, that allows tapping the 12V and 3.3V power lines.  Also, the riser cable should be of sufficient quality, to avoid PCIe bandwidth loss due to poor signalling and to ensure PCIe gen 4 compatibility.  The [LINKUP {30 cm} Shielded Twin-axial Riser Cable](https://linkup.one/linkup-30-cm-pcie-4-0-3-0-16x-extreme-shielded-twin-axial-riser-cable-port-extension-pcie-card-90-degree-socket/) will do.  The 12V power line can provide up to 65W (5.4A), so the ACS712-20 is a suitable current sensor.  The 3.3V power line can provide up to 10W (3.0A); this can best be measured using an ACS712-5 current sensor. Because the STM32F407VG operates at 3.3 V, the current has to be inversed. So the current sensors have to be flipped when they are installed.

To install the sensors on the riser cable, the wires have to be cut. Below is a table indicating which line is which. The 20 A sensor needs to be connected to all the 12 V lines. Start counting the lines from the outer most side (right side in the picture above), because the riser cable has 12 wires instead of 11. The 5 A sensor needs to be connected to all the 3.3 V lines, the AUX one too.

![Imgur](https://i.imgur.com/dfpgjxF.png)

A third current sensor is needed to measure the power of an external 12V power cable.  We advise to not cut the PCIe power cables of the power supply directly, but to use an extension cable and cut the 12V wires of the extension cable.  Only cut the 12V wires; the earth wires should not be cut.  Connect both ends of the cut cable to the current sensor.  If the current flows in the wrong direction, PowerSensor will measure negative power, so make sure that the cable is connected correctly.

The voltage output of the current sensor should be connected to any of the ports PA0-A4. The current sensor can be powered by STM32F407VG, by connecting it to a breadboard or any other power line supplied by the board.

Finally, connect the STM32F407VG's mini-USB to a USB port of the host system. Then plug in the micro-USB to the host. The mini-USB connection to the host is only required for the installing of the firmware, afterwards it only supplies the board with power. Further communication with the board is done via the micro-USB port. After making sure that everything is connected correctly, turn on the host system.

## Installing the firmware TOFIX
Make sure that the Arduino tools are installed on host system.  On Ubuntu, one needs the `arduino-core` and `arduino-mk` packages.  If necessary, adapt `Arduino/Makefile` to configure the right Arduino variant.  The device typically appears as `/dev/ttyACM0` after it is connected to the host.  Then, do `make upload` to build and install the firmware on the Arduino.  The PowerSensor will not work properly until its is configured.