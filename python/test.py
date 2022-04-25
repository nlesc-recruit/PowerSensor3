#!/usr/bin/env python
import powersensor
from time import sleep
import IPython

if __name__ == '__main__':
    ps = powersensor.PowerSensor("/dev/cu.usbmodem386A367F32371")
    ps.dump("output.txt")
    sleep(1)
    state1 = ps.read()
    ps.mark('Q')
    sleep(2)
    state2 = ps.read()

    keys = ["consumed_energy", "current", "voltage", "time_at_read"]
    units = dict(zip(keys, ["J", "A", "V", "s"]))
    print("State values:")
    for key in keys:
        value = getattr(state1, key)
        if isinstance(value, list):
            formatted_value = " ".join([f"{v:.2f}" for v in value])
        else:
            formatted_value = f"{value:.2f}"
        print(f'{key}: {formatted_value} {units[key]}')

    print("Comparing two states:")
    funcs = ["Joules", "seconds", "Watt", "Volt", "Ampere"]
    for func in funcs:
        if func == 'seconds':
            value = getattr(powersensor, func)(state1, state2)
        else:
            value = getattr(powersensor, func)(state1, state2, 0)  # first sensor
        print(f'{value:.2f} {func}')

    #IPython.embed()
