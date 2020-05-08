#!/bin/bash
echo "Compiling sketch: test"
arduino-cli compile --fqbn STM32:stm32:Disco:pnum=DISCO_F407VG,usb=CDCgen test
echo "Compiling complete"
echo "Uploading sketch: test, to: /dev/ttyACM0"
arduino-cli upload -p /dev/ttyACM0 --fqbn STM32:stm32:Disco:pnum=DISCO_F407VG,usb=CDCgen test
echo "Uploading complete"
echo "Waiting 25 (s) for device reset"
sleep 25
echo "Opening serial port with PlatformIO"
pio device monitor -p /dev/ttyACM1
