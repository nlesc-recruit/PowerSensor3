.PHONY = all compile upload

all: compile upload

compile:
	@echo "compile"
	arduino-cli compile --fqbn STM32:stm32:Disco:pnum=DISCO_F407VG,usb=CDCgen powersensor

upload:
	@echo "upload"
	arduino-cli upload -p /dev/ttyACM1 --fqbn STM32:stm32:Disco:pnum=DISCO_F407VG,usb=CDCgen powersensor

