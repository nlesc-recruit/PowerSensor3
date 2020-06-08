# Installation guide
## Installing the PowerSensor
Before starting, turn off and unplug the power of the host system.  An Arduino Uno (or similar clone) is at the heart of the PowerSensor.  The microcontroller on this board collects the sensor values, interprets them, and communicates them to the host via USB.  An optional LCD shield (with 16x2 display) shows the instantaneous power consumption of the (PCIe) device to be measured.

PowerSensor measures currents, using external current sensors (like those from the ACS712 series).  The current sensor, in this case a Hall-effect sensor, measures the current that flows through a wire, and converts the input current to an (analogue) voltage that is directly proportional to the current.  This voltage is sampled by the built-in ADC converter of the Arduino.  PowerSensor assumes that no current (0 Ampere) translates to a 2.5V signal.

To measure the power of a PCIe device, a riser card is necessary, so that the current drawn from the PCIe slot can be measured.  An appropriate rised card is necessary, that allows tapping the 12V and 3.3V power lines.  Also, the riser card should be of sufficient quality, to avoid PCIe bandwidth loss due to poor signalling.  The Adexelec PEXP16-EX will do.  The 12V power line can provide up to 65W (5.4A), so the ACS712-20 is a suitable current sensor.  The 3.3V power line can provide up to 10W (3.0A); this can best be measured using an ACS712-5 current sensor.

A third current sensor is needed to measure the power of an external 12V power cable.  We advise to not cut the PCIe power cables of the power supply directly, but to use an extension cable and cut the 12V wires of the extension cable.  Only cut the 12V wires; the earth wires should not be cut.  Connect both ends of the cut cable to the current sensor.  If the current flows in the wrong direction, PowerSensor will measure negative power, so make sure that the cable is connected correctly.

The voltage output of the current sensor should be connected to any of the ports A1-A5 (A0 is used by the buttons of the LCD shield).  The current sensor can be powered by Arduino bord, by connecting it to the 5V and earth connectors.

Finally, connect the Arduino to a USB port of the host system.  After making sure that everything is connected correctly, turn on the host system.

## Installing the firmware
Make sure that the Arduino tools are installed on host system.  On Ubuntu, one needs the `arduino-core` and `arduino-mk` packages.  If necessary, adapt `Arduino/Makefile` to configure the right Arduino variant.  The device typically appears as `/dev/ttyACM0` after it is connected to the host.  Then, do `make upload` to build and install the firmware on the Arduino.  The PowerSensor will not work properly until its is configured.