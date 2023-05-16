#!/usr/bin/env python

from pathlib import Path
import subprocess

import yaml


if __name__ == '__main__':
    try:
        raw_config = subprocess.Popen(['arduino-cli', 'config', 'dump'], stdout=subprocess.PIPE).communicate()[0]
    except FileNotFoundError as e:
        raise ValueError('Failed to run arduino-cli, is it installed?') from e
    config = yaml.safe_load(raw_config)
    data_dir = Path(config['directories']['data'])

    stm32_dir = data_dir / 'packages' / 'STMicroelectronics' / 'hardware' / 'stm32'
    if not stm32_dir.is_dir():
        raise ValueError('stm32duino directory not found, is it installed?')
    # the subfolders of stm32 have the stm32duino version as name
    # pick the latest version
    stm32_dir = list(stm32_dir.glob('*'))[-1]
    print(stm32_dir)
