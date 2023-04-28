#!/usr/bin/env python
import sys
from time import sleep

import powersensor

if __name__ == '__main__':
    try:
        device = sys.argv[1]
    except IndexError:
        device = "/dev/cu.usbmodem386A367F32371"
    print(f"Using device at {device}")

    ps = powersensor.PowerSensor(device)
    ps.dump("output.txt")
    sleep(1)
    state1 = ps.read()
    ps.mark('Q')
    sleep(2)
    state2 = ps.read()

    keys = ["consumed_energy", "voltage", "current", "time_at_read"]
    units = dict(zip(keys, ["J", "V", "A", "s"]))
    print("\nSingle state values:")
    for key in keys:
        value = getattr(state1, key)
        if isinstance(value, list):
            formatted_value = " ".join([f"{v:.2f}" for v in value])
        else:
            formatted_value = f"{value:.2f}"
        print(f'{key}: {formatted_value} {units[key]}')

    print("\nComparing two states:")
    print(f"Passed time: {powersensor.seconds(state1, state2):.2f} s")
    print(f"Total energy: {powersensor.Joules(state1, state2, -1):.2f} J")
    print("\nTotals/averages per sensor pair:")
    funcs = ["Joules", "Watt"]
    units = dict(zip(funcs, ["J (total)", "W (avg)"]))
    for func in funcs:
        values = [getattr(powersensor, func)(state1, state2, i) for i in range(4)]
        formatted_value = ' '.join([f'{item:.2f}' for item in values])
        print(f'{formatted_value} {units[func]}')

