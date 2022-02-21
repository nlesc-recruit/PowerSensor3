CXX ?=				g++
CXXFLAGS =			-std=c++11 -O2 -g -pthread -fopenmp -fPIC
INC = -Ihost/include
LIB = -Lhost/lib -lPowerSensor

BOARD =				DISCO_F407VG
USB =				CDCgen
DEVICE =            STMicroelectronics:stm32:Disco
FQBN =				$(DEVICE):pnum=$(BOARD),usb=$(USB)

ifeq ($(OS), Darwin)
	PORT =			/dev/cu.usbmodem144103
else
	PORT =			/dev/ttyACM0
endif

host/obj/%.o: host/src/%.cc
	-mkdir -p host/obj
	$(CXX) -c $(CXXFLAGS) $(INC) $< -o $@

all:: lib bin device

bin:: host/bin/test_ps

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
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) -i device/$(BOARD)/PowerSensor/build/$(subst :,.,$(DEVICE))/PowerSensor.ino.bin

clean:
	$(RM) -r host/bin host/lib host/obj device/$(BOARD)/PowerSensor/build
