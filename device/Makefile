# Makefile for device code
DEV ?= F401

ifeq ($(DEV), F401)
# BLack Pill F401CCU6
BOARD =        BLACKPILL_F401CC
DEVICE =       STMicroelectronics:stm32:GenF4
FQBN =	       $(DEVICE):pnum=$(BOARD),usb=CDCgen,upload_method=dfuMethod,opt=o3std
else ifeq ($(DEV), F411)
# Black Pill F411CEU6
BOARD =        BLACKPILL_F411CE
DEVICE =       STMicroelectronics:stm32:GenF4
FQBN =	       $(DEVICE):pnum=$(BOARD),usb=CDCgen,upload_method=dfuMethod,opt=o3std
else ifeq ($(DEV), F407)
# Discovery F407
BOARD =			DISCO_F407VG
DEVICE =        STMicroelectronics:stm32:Disco
FQBN =          $(DEVICE):pnum=$(BOARD),usb=CDCgen,upload_method=dfuMethod,xusb=FS,opt=o3std
else
$(error Unknown device $(DEV), allowed devices are F401 (default), F411, and F407)
endif


all: device

device:
	arduino-cli compile -e --fqbn $(FQBN) PowerSensor --libraries PowerSensor/Libraries --build-property compiler.cpp.extra_flags="$(FLAGS)"

upload: device
	arduino-cli upload $(OPT) --fqbn $(FQBN) -i PowerSensor/build/$(subst :,.,$(DEVICE))/PowerSensor.ino.bin

fontmap:
	../python/create_fontmaps.py > PowerSensor/fontMaps.hpp

.PHONY: all
