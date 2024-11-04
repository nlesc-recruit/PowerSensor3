# User guide

## Device location and permissions
PowerSensor3 is detected as a serial device, under Linux typically as `/dev/ttyACM0`. By default, users need to be added to the `dialout` group to be able to access the device.

## Monitor power use of an existing application
Adapting an application to use the library is not obligatory; the `psrun` utility can monitor the power use of a device during the execution of an application that does not use the library. Run `psrun -h` for more options
```
psrun <application>
psrun version 1.3.2

< application output >
5.02273 s, 144.12 J, 28.6935 W
```

## Using the host library
The host library is a small C++ library that can be used by applications to measure the power used by some (PCIe) device during some time interval.  The interface (declared in `PowerSensor.hpp`) looks like this:
```
double Joules(const State &first, const State &second, int pairID=-1);
double Watt(const State &first, const State &second);
double seconds(const State &first, const State &second, int pairID=-1);

class PowerSensor {
  public:
    ...
    State read();
};
```
and can be used as follows:
```
#include <PowerSensor.hpp>

using namespace PowerSensor3;

int main()
{
    PowerSensor sensor("/dev/ttyACM0");
    State start = sensor.read();
    ...
    State stop = sensor.read();
    std::cout << "The computation took " << Joules(start, stop) << 'J' << std::endl;
}
```

This way, the power consumption during some time interval can be used. The `Joules` function gives an accurate total eneryg usage. `Watt` gives the _average_ power usage over the given time interval.

Another way to use the PowerSensor is to produce a stream of sensor values on file (in ASCII). Simply provide the name of the file as second (optional) argument to the constructor of PowerSensor.  A separate, light-overhead thread will be started that continuously monitors the PowerSensor state. Both methods can be used at the same time.

For further usage, inspect the `PowerSensor.hpp` file or have a look at the [API documentation](https://nlesc-recruit.github.io/PowerSensor3).

## Python interface
An optional Python interface is available. See [here](INSTALLATION_HOST.md#python-bindings) for installation instructions. The `PowerSensor` and `State` objects can be accessed from Python, as well as the `Joules`, `Watt`, and `seconds` functions. Example usage:

```python
import powersensor as ps

sensor = ps.PowerSensor('/dev/ttyACM0')
start = ps.read()
< application code here>
stop = ps.read()

print(f'{ps.Joules(start, stop):.2f}J, {ps.seconds(start, stop):.2f}s, {ps.Watt(start, stop):.2f}W')
```
