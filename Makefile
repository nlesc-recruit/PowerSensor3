OS = $(shell uname -s)
ARCH = $(shell uname -m)
CXX ?=				g++
CXXFLAGS =			-std=c++11 -O2 -g -pthread -fopenmp
BOARD =				DISCO_F407VG
USB =				CDCgen
FQBN =				STMicroelectronics:stm32:Disco:pnum=$(BOARD),usb=$(USB)

ifeq ($(OS), Darwin)
	PORT =			/dev/cu.usbmodem141203
else
	PORT =			/dev/ttyACM0
endif


host/obj/$(ARCH)/%.o:		host/%.cc
				@mkdir -p host/obj/$(ARCH)
				$(CXX) -c $(CXXFLAGS) $< -o $@

all::			host arduino

host/lib/$(ARCH)/libPowerSensor.a: 	host/obj/$(ARCH)/PowerSensor.o
					-mkdir -p host/lib/$(ARCH)
					$(AR) cr $@ $<

host/bin/$(ARCH)/psconfig:	host/obj/$(ARCH)/psconfig.o host/lib/$(ARCH)/libPowerSensor.a
				-mkdir -p host/bin/$(ARCH)
				$(CXX) $(CXXFLAGS) host/obj/$(ARCH)/psconfig.o -Lhost/lib/$(ARCH) -lPowerSensor -o $@

host/bin/$(ARCH)/psrun:		host/obj/$(ARCH)/psrun.o host/lib/$(ARCH)/libPowerSensor.a
				-mkdir -p host/bin/$(ARCH)
				$(CXX) $(CXXFLAGS) host/obj/$(ARCH)/psrun.o -Lhost/lib/$(ARCH) -lPowerSensor -o $@

host/bin/$(ARCH)/pstest:	host/obj/$(ARCH)/pstest.o host/lib/$(ARCH)/libPowerSensor.a
				-mkdir -p host/bin/$(ARCH)
				$(CXX) $(CXXFLAGS) host/obj/$(ARCH)/pstest.o -Lhost/lib/$(ARCH) -lPowerSensor -o $@

host/bin/$(ARCH)/psraw:	host/obj/$(ARCH)/psraw.o host/lib/$(ARCH)/libPowerSensor.a
				-mkdir -p host/bin/$(ARCH)
				$(CXX) $(CXXFLAGS) host/obj/$(ARCH)/psraw.o -Lhost/lib/$(ARCH) -lPowerSensor -o $@

host/obj/$(ARCH)/psconfig.o:	host/psconfig.cc host/PowerSensor.h

host/obj/$(ARCH)/psrun.o:	host/psrun.cc host/PowerSensor.h

host/obj/$(ARCH)/pstest.o:	host/pstest.cc host/PowerSensor.h

host/obj/$(ARCH)/psraw.o:	host/psraw.cc host/PowerSensor.h

host/obj/$(ARCH)/PowerSensor.o: host/PowerSensor.cc host/PowerSensor.h host/Semaphore.h

host::			host/lib/$(ARCH)/libPowerSensor.a\
				host/bin/$(ARCH)/psconfig\
				host/bin/$(ARCH)/psrun\
				host/bin/$(ARCH)/pstest\
				host/bin/$(ARCH)/psraw

arduino::
				arduino-cli compile -e --fqbn $(FQBN) device/$(BOARD)/PowerSensor

upload::			all
				arduino-cli upload -p $(PORT) --fqbn $(FQBN) device/$(BOARD)/PowerSensor

clean:
				$(RM) -r device/$(BOARD)/PowerSensor/PowerSensor.STM32.stm32.Disco.*
				$(RM) -r host/bin/$(ARCH) host/lib/$(ARCH) host/obj/$(ARCH)
