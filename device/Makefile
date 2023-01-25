# Makefile for device code

# For Discovery F407
#BOARD =			DISCO_F407VG
#DEVICE =           STMicroelectronics:stm32:Disco
#FQBN =				$(DEVICE):pnum=$(BOARD),usb=$(USB)
#OPT =              -p /dev/ttyACM0

# For Black Pill F401CCU6
BOARD =				BLACKPILL_F401CC
DEVICE =            STMicroelectronics:stm32:GenF4
FQBN =				$(DEVICE):pnum=$(BOARD),usb=$(USB),upload_method=dfuMethod

USB =				CDCgen

all: device

device:
	arduino-cli compile -e --fqbn $(FQBN) $(BOARD)/PowerSensor --libraries $(BOARD)/PowerSensor/Libraries

upload: device
	arduino-cli upload $(OPT) --fqbn $(FQBN) -i $(BOARD)/PowerSensor/build/$(subst :,.,$(DEVICE))/PowerSensor.ino.bin

.PHONY: all
