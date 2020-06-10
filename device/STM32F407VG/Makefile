# Usage:
# make		# compile and upload file
# make clean	# remove ALL files

.PHONY 	= all compile upload

BOARD 	= DISCO_F407VG
USB 	= CDCgen
PORT	= /dev/ttyACM1
FQBN 	= STM32:stm32:Disco:pnum=$(BOARD),usb=$(USB)
SKETCH 	= powersensor

all: compile upload

compile:
	arduino-cli compile --fqbn ${FQBN} ${SKETCH}

upload:
	arduino-cli upload -p ${PORT} --fqbn ${FQBN} ${SKETCH}

clean:
	rm -rvf powersensor/powersensor.STM32.stm32.Disco.*
