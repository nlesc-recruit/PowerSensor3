#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    dt, current, voltage = np.loadtxt('out.txt', dtype=int, unpack=True)

    expected_dt = 1000. / 159.09

    print("Found {len(dt)} data points")
    print(f"Expected dt: {expected_dt} us")

    print(f'{dt.min()=} {dt.mean()=} {dt.max()=}')
    print(f'{np.median(dt)=} {dt.std()=}')

    plt.plot(dt, c='k', marker='o', ls='')
    plt.axhline(expected_dt, c='r', ls='--')
    plt.xlim(0, min(len(dt), 1000))
    plt.ylim(0, 2*dt.mean())
    plt.xlabel('Time step')
    plt.ylabel('Interval ($\mu$s)')
    plt.show()

