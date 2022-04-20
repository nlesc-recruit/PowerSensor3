#!/usr/bin/env python
import powersensor
from time import sleep

if __name__ == '__main__':
    ps = powersensor.PowerSensor("/dev/cu.usbmodem386A367F32371")
    sleep(1)
    state = ps.read()
    print(state.consumed_energy)
