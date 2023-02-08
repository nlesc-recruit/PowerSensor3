# Makefile for device code

USB =				CDCgen

# For Discovery F407
BOARD =			DISCO_F407VG
DEVICE =           STMicroelectronics:stm32:Disco
FQBN =				$(DEVICE):pnum=$(BOARD),usb=$(USB),upload_method=MassStorage,xusb=FS
#OPT =              -p /dev/ttyACM0
OPT =              -p /dev/cu.usbmodem142403

# For Black Pill F401CCU6
#BOARD =				BLACKPILL_F401CC
#DEVICE =            STMicroelectronics:stm32:GenF4
#FQBN =				$(DEVICE):pnum=$(BOARD),usb=$(USB),upload_method=dfuMethod


all: device

device:
	arduino-cli compile -e --fqbn $(FQBN) $(BOARD)/PowerSensor --libraries $(BOARD)/PowerSensor/Libraries

upload: device
	arduino-cli upload $(OPT) --fqbn $(FQBN) -i $(BOARD)/PowerSensor/build/$(subst :,.,$(DEVICE))/PowerSensor.ino.bin

.PHONY: all
