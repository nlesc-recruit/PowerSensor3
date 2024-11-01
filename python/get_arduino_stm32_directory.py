#!/usr/bin/env python3

from pathlib import Path
from shutil import which
import subprocess

import yaml


if __name__ == '__main__':

    arduino_cli = which('arduino-cli')
    if arduino_cli is None:
        raise ValueError("arduino-cli not found")

    raw_config = subprocess.Popen([arduino_cli, 'config', 'dump'], stdout=subprocess.PIPE).communicate()[0]
    config = yaml.safe_load(raw_config)
    try:
        data_dir = Path(config['directories']['data'])
    except KeyError:
        raise KeyError("Arduino data directory not found in arduino-cli config file")

    stm32_dir = data_dir / 'packages' / 'STMicroelectronics' / 'hardware' / 'stm32'
    if not stm32_dir.is_dir():
        raise ValueError('stm32duino directory not found, is it installed?')
    # the subfolders of stm32 have the stm32duino version as name
    # pick the latest version
    stm32_dir = list(stm32_dir.glob('*'))[-1]
    print(stm32_dir)
