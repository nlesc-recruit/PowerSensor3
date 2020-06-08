# User guide
## Configuring the PowerSensor
`psconfig` configures the PowerSensor.  The parameters that can be set are the voltages of the lines to be measured, the type of the current sensor, and the null level of the ADC.  The parameters are stored on the device's EEPROM, so the device needs to be configured only once.

There is native support for the ACS712 series current sensors.  Other sensor types may work as well (see the `-t` option below).

PowerSensor supports up to 5 current sensors.  All current sensor ports must be configured, even if not all of them are used.

`psconfig` accepts the parameters as described below.  The order of parameters is important.  Further on, we give examples of full configuration commands for a typical PowerSensor setup.

```
usage: ./bin/x86_64/psconfig [-d device] [-s sensor] [-t type] [-v volt] [-a | -n nullLevel] [-o] [-p]
-d selects the device (default: /dev/ttyACM0)
-s selects the sensor (0-4)
-t selects the sensor type, (one of ACS712-5, ACS712-20, ACS712-30, or a Volt-Per-Ampere value)
-v sets the voltage level
-a auto-calibrates the null level
-n manually sets the null level (the amount of Watt to be added)
-o turns off a sensor
-p prints configured values
```

Parameters that are not specified are left unmodified on the device.

A typical configuration of the PowerSensor is to measure a PCIe device like a
GPU, with current sensor 0 measuring the 12V PCIe slot current, sensor 1
measuring the 3.3V PCIe slot current, and sensor 2 measuring the 12V external
cable current.  A typically way to configure the PowerSensor is as follows:

```
./bin/x86_64/psconfig -d/dev/ttyACM0 -s0 -tACS712-20 -v12 -n0 -s1 -tACS712-5 -v3.3 -n0 -s2 -tACS712-20 -v12 -n0 -s3 -o -s4 -o -p
```

The `-n` values may be adjusted to get the right null levels, depending on the local magnetic field.  An easier way to calibrate them, is to fully turn of the host system power (so that no current is flowing through the current sensors), and to configure the PowerSensor from another machine (by temporarily connecting the USB cable to that other machine).  In this case, the null levels can be configured automatically:

```
$ ./bin/x86_64/psconfig -d/dev/ttyACM0 -s0 -tACS712-20 -v12 -a -s1 -tACS712-5 -v3.3 -a -s2 -tACS712-20 -v12 -a -s3 -o -s4 -o -p
```


## Testing the PowerSensor
To see if the PowerSensor works correctly, one can either use the `-p` option of `psconfig`:
```
$ ./bin/x86_64/psconfig -h
sensor: 0, volt: 12 V, type: ACS712-20, null level: -1.66026 W, current usage: 11.5142 W
sensor: 1, volt: 3.3 V, type: ACS712-5, null level: 0.0223578 W, current usage: 0.262142 W
sensor: 2, volt: 12 V, type: ACS712-20, null level: -1.75054 W, current usage: 11.19 W
sensor: 3, off
sensor: 4, off
```

Or use the `pstest` utility to measure and report energy consumption for a few seconds:
```
$ ./bin/x86_64/pstest
exp. time: 0.0004 s, measured: 0.000476902 s, 0.0597194 J, 125.224 W
exp. time: 0.0008 s, measured: 0.000911695 s, 0.0190849 J, 20.9334 W
exp. time: 0.0016 s, measured: 0.00173528 s, 0.0387305 J, 22.3195 W
exp. time: 0.0032 s, measured: 0.00333613 s, 0.0786753 J, 23.5828 W
exp. time: 0.0064 s, measured: 0.00653597 s, 0.150446 J, 23.0182 W
exp. time: 0.0128 s, measured: 0.0129376 s, 0.295892 J, 22.8707 W
exp. time: 0.0256 s, measured: 0.0257391 s, 0.585381 J, 22.7429 W
exp. time: 0.0512 s, measured: 0.0512283 s, 1.1743 J, 22.8738 W
exp. time: 0.1024 s, measured: 0.102544 s, 2.36651 J, 23.078 W
exp. time: 0.2048 s, measured: 0.204941 s, 4.70485 J, 22.9571 W
exp. time: 0.4096 s, measured: 0.409762 s, 9.39299 J, 22.923 W
exp. time: 0.8192 s, measured: 0.819362 s, 18.8299 J, 22.9811 W
exp. time: 1.6384 s, measured: 1.63861 s, 37.6751 J, 22.9921 W
exp. time: 3.2768 s, measured: 3.27695 s, 75.2596 J, 22.9663 W
```

## Monitor power use of an existing application
Adapting an application to use the library is not obligatory; the `psrun` utility can monitor the power use of a device during the execution of an application that does not use the library:
```
./bin/x86_64/pstest <application>
< application output >
24.0161 s, 5537,22 J, 230.563 W
```

## Using the host library
The host library is a small C++ library that can be used by applications to measure the power used by some (PCIe) device during some time interval.  The interface (declared in `PowerSensor.h`) looks like this:
```
class PowerSensor {
  public:
    ...
    State read();

    static double Joules(const State &first, const State &second);
    static double Watt(const State &first, const State &second);
    static double seconds(const State &first, const State &second);
};
```
and can be used as follows:
```
#include <PowerSensor.h>

using namespace powersensor;

int main()
{
    PowerSensor sensor("/dev/ttyACM0");
    PowerSensor::State start = sensor.read();
    ...
    PowerSensor::State stop = sensor.read();
    std::cout << "The computation took " << PowerSensor::Joules(start, stop) << 'J' << std::endl;
}
```

This way, the power consumption during some time interval can be used.

Another way to use the PowerSensor is to produce a stream of sensor values on file (in ASCII). Simply provide the name of the file as second (optional) argument to the constructor of PowerSensor.  A separate, light-overhead thread will be started that continuously monitors the PowerSensor state. Both methods can be used at the same time.
