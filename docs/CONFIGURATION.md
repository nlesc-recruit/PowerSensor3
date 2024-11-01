# Configuration
Please make sure that the PowerSensor is installed correctly by completing the [device installation guide](INSTALLATION_DEVICE.md) and [host library installation guide](INSTALLATION_HOST.md).

## Configuring PowerSensor
`psconfig` configures PowerSensor. For each of the up to 8 connected sensors, the type and calibration values can be set. These values are stored on the device's emulated EEPROM, so configuration only needs to happend once. `psconfig` can also toggle the attached display and print the current sensor values.

`psconfig` accepts the parameters as described below.  The order of parameters is important.  Further on, we give examples of full configuration commands for a typical PowerSensor setup.

```
$ psconfig -h
usage: psconfig [-h] [-d device] [-s sensor] [-t type] [-m name] [-a | -v volt] [-n sensitivity] [-x polarity] [-o on/off] [-p]
-h prints this help
-d selects the device (default: /dev/ttyACM0)
-s selects the sensor (0-8)
-t sets the sensor type. This also sets the sensitivity to the default value if the sensor is of a type known to this programme (see list at the bottom of this help).
-m sets the sensor pair name. Setting this for either of the sensors of a pair sets the same pair name for both sensors
-v sets the reference voltage level
-a automatically calibrate vref of the current sensor. The input to the sensor must be zero volt or ampere
-n set the sensitivity in mV/A for current sensors (even sensors) or unitless gain for voltage sensors (odd sensors)
-x sets the polarity of a sensor. 1 for normal, -1 for inverted
-o turns a sensor on (1) or off (0)
-p prints configured values
example: psconfig -d /dev/ttyACM0 -s 0 -t MLX10 -v 1.65 -o 1 -s 1 -t voltage0 -v 0 -n 0.95 -o 1 -p
Known current sensor types: MLX10, MLX20, MLX50, MLX75.
```

Parameters that are not specified are left unmodified on the device.

The `-v` values may be adjusted to get the right null levels, depending on the local magnetic field.  An easier way to calibrate them, is to fully turn of the host system power (so that no current is flowing through the current sensors), and to configure the PowerSensor from another machine (by temporarily connecting the USB cable to that other machine).  In this case, the null levels can be configured automatically. Example when four sensors are connected:

```
$ psconfig -d/dev/ttyACM0 -s 0 -a -s 1 -a -s2 -a -s 3 -a
```

This feature is especially useful for the current sensors. For voltage sensors, the reference voltage is typically (very close to) zero.


## Testing the PowerSensor
To see if the PowerSensor works correctly, one can either use the `-p` option of `psconfig`:
```
$ psconfig -p
psconfig version 1.3.2

sensor 0, pair 0 (current): type: MLX10, name: NA, Vref: 1.636 V, Sensitivity: 120 mV/A, polarity: 1, Status: off
sensor 1, pair 0 (voltage): type: voltage, name: NA, Vref: 0 V, Gain: 0.091, polarity: 1, Status: off
sensor 2, pair 1 (current): type: NA, name: NA, Vref: 0 V, Sensitivity: 0 mV/A, polarity: 1, Status: off
sensor 3, pair 1 (voltage): type: NA, name: NA, Vref: 0 V, Gain: 0, polarity: 1, Status: off
sensor 4, pair 2 (current): type: NA, name: NA, Vref: 0 V, Sensitivity: 0 mV/A, polarity: 1, Status: off
sensor 5, pair 2 (voltage): type: NA, name: NA, Vref: 0 V, Gain: 0, polarity: 1, Status: off
sensor 6, pair 3 (current): type: MLX10, name: Jetson, Vref: 1.62247 V, Sensitivity: 120 mV/A, polarity: 1, Status: on
sensor 7, pair 3 (voltage): type: voltage, name: Jetson, Vref: 0 V, Gain: 0.0877, polarity: 1, Status: on
Current usage pair 0 (sensors 0, 1): 0 W
Current usage pair 1 (sensors 2, 3): 0 W
Current usage pair 2 (sensors 4, 5): 0 W
Current usage pair 3 (sensors 6, 7): 0.0175141 W
Total usage: 0.0175141 W
```

Or use the `pstest` utility to measure and report energy consumption for a few seconds. Run `pstest -h` for more options.
```
$ pstest
pstest version 1.3.2

exp. time: 0.0002 s, measured: 0.00571626 s, 0.00114577 J, 0.200441 W
exp. time: 0.0004 s, measured: 0.000343234 s, 0.0112714 J, 32.8388 W
exp. time: 0.0008 s, measured: 0.00090007 s, 0.0293691 J, 32.6298 W
exp. time: 0.0016 s, measured: 0.00177352 s, 0.0566265 J, 31.929 W
exp. time: 0.0032 s, measured: 0.00359161 s, 0.118366 J, 32.9563 W
exp. time: 0.0064 s, measured: 0.00640616 s, 0.195132 J, 30.4601 W
exp. time: 0.0128 s, measured: 0.0132216 s, 0.4025 J, 30.4427 W
exp. time: 0.0256 s, measured: 0.0257813 s, 0.776679 J, 30.1257 W
exp. time: 0.0512 s, measured: 0.0517088 s, 1.52741 J, 29.5387 W
exp. time: 0.1024 s, measured: 0.10309 s, 2.97431 J, 28.8516 W
exp. time: 0.2048 s, measured: 0.20566 s, 5.87204 J, 28.5521 W
exp. time: 0.4096 s, measured: 0.409726 s, 11.7305 J, 28.6301 W
exp. time: 0.8192 s, measured: 0.784317 s, 22.3385 J, 28.4815 W
exp. time: 1.6384 s, measured: 1.67394 s, 47.6963 J, 28.4935 W
exp. time: 3.2768 s, measured: 3.27701 s, 93.377 J, 28.4946 W
```

## Next steps
Usage of the PowerSensor is described in [this guide](USERGUIDE.md).
