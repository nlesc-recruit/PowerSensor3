CXX ?=				g++
CXXFLAGS =			-std=c++11 -O2 -g -pthread -fopenmp -Wall
INC = -Ihost/include
LIB = -Lhost/lib -lPowerSensor

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

host/obj/%.o: host/src/%.cc
	-mkdir -p host/obj
	$(CXX) -c $(CXXFLAGS) $(INC) $< -o $@

all: lib bin device python

bin: lib host/bin/test_ps host/bin/psconfig

lib: host/obj/PowerSensor.o host/obj/sensors.o
	-mkdir -p host/lib
	$(AR) rcs host/lib/libPowerSensor.a $^
	$(CXX) $(CXXFLAGS) -shared -fPIC -install_name @rpath/libPowerSensor.so -o host/lib/libPowerSensor.so $^

host/bin/%: host/obj/%.o
	-mkdir -p host/bin
	$(CXX) $(CXXFLAGS) $(INC) $(LIB) -Wl,-rpath,$(shell pwd)/host/lib $< -o $@

device:
	arduino-cli compile -e --fqbn $(FQBN) device/$(BOARD)/PowerSensor

upload: device
	arduino-cli upload $(OPT) --fqbn $(FQBN) -i device/$(BOARD)/PowerSensor/build/$(subst :,.,$(DEVICE))/PowerSensor.ino.bin

python:
	$(MAKE) -C $@

clean:
	$(RM) -r host/bin host/lib host/obj device/$(BOARD)/PowerSensor/build
	$(MAKE) -C python clean

.PHONY: all python
