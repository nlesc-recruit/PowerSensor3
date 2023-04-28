# BLack Pill F401CCU6
BOARD =        BLACKPILL_F401CC
DEVICE =       STMicroelectronics:stm32:GenF4
#FQBN =	       $(DEVICE):pnum=$(BOARD),usb=CDCgen,upload_method=dfuMethod,opt=o3std
FQBN =	       $(DEVICE):pnum=$(BOARD),usb=CDCgen,upload_method=dfuMethod

all: device

device:
	arduino-cli compile -e --fqbn $(FQBN) ./display_bitmap --libraries ./display_bitmap/Libraries --build-property compiler.cpp.extra_flags="$(FLAGS)"

upload: device
	arduino-cli upload $(OPT) --fqbn $(FQBN) -i ./display_bitmap/build/$(subst :,.,$(DEVICE))/display_bitmap.ino.bin

.PHONY: all
