#!/usr/bin/env python3
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    fname = sys.argv[1]
    # dt, current, voltage = np.loadtxt(fname, dtype=int, unpack=True)
    t, current, voltage, dt = np.loadtxt(fname, dtype=int, unpack=True)

    # first dt is always zero, remove it to not skew results
    dt = dt[1:]


    # measurement is 5 seconds
    actual_dt = 5.0e6 / len(dt)

    expected_dt = 1000. / 68.18

    print(f"Found {len(dt)} data points")
    print(f"Expected dt: {expected_dt:.2f} us")
    print(f"Actual average dt on host: {actual_dt:.2f} us")

    print(f'{dt.min()=:.0f} {dt.mean()=:.2f} {dt.max()=:.0f}')
    print(f'{np.median(dt)=:.0f} {dt.std()=:.2f}')

    plt.plot(dt, c='k', marker='o', ls='')
    plt.axhline(expected_dt, c='r', ls='--')
    plt.xlim(0, min(len(dt), 1000))
    plt.ylim(0, 2*dt.mean())
    plt.xlabel('Time step')
    plt.ylabel('Interval ($\mu$s)')
    plt.title(os.path.splitext(fname)[0])
    plt.show()

    import IPython; IPython.embed()
