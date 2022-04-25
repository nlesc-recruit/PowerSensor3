CXX ?=				g++
CXXFLAGS =			-std=c++11 -O2 -g -pthread -fopenmp -fPIC
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

all:: lib bin device

bin:: lib host/bin/test_ps host/bin/psconfig

lib: host/obj/PowerSensor.o host/obj/sensors.o
	-mkdir -p host/lib
	$(AR) rcs host/lib/libPowerSensor.a $^
	$(CXX) -shared -fopenmp -o host/lib/libPowerSensor.so $^

host/bin/%: host/obj/%.o
	-mkdir -p host/bin
	$(CXX) $(CXXFLAGS) $(INC) $(LIB) $< -o $@

device::
	arduino-cli compile -e --fqbn $(FQBN) device/$(BOARD)/PowerSensor

upload:: device
	arduino-cli upload $(OPT) --fqbn $(FQBN) -i device/$(BOARD)/PowerSensor/build/$(subst :,.,$(DEVICE))/PowerSensor.ino.bin

emulator: emulator/src/emulator.cc emulator/src/device.cc
	-mkdir -p emulator/bin
	$(CXX) $(CXXFLAGS) $(INC) -Iemulator/include -lutil -o emulator/bin/emulator $^

clean:
	$(RM) -r host/bin host/lib host/obj device/$(BOARD)/PowerSensor/build
