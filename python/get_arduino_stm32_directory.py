#!/usr/bin/env python3

from pathlib import Path
from shutil import which
import subprocess

import yaml


if __name__ == '__main__':

    arduino_cli = which('arduino-cli')
    if arduino_cli is None:
        raise ValueError("arduino-cli not found")

    data_dir = Path(subprocess.Popen([arduino_cli, 'config', 'get', 'directories.data'], stdout=subprocess.PIPE).communicate()[0].decode().strip())
    if not data_dir.is_dir():
        raise ValueError(f"Arduino data directory ({data_dir})) does not exist")

    stm32_dir = data_dir / 'packages' / 'STMicroelectronics' / 'hardware' / 'stm32'
    if not stm32_dir.is_dir():
        raise ValueError('stm32duino directory not found, is it installed?')
    # the subfolders of stm32 have the stm32duino version as name
    # pick the latest version
    stm32_dir = list(stm32_dir.glob('*'))[-1]
    print(stm32_dir)
