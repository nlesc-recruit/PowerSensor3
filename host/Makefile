ARCH=$(shell arch)
CXX=			g++
CXXFLAGS=		-std=c++11 -O2 -g -pthread -fopenmp

obj/$(ARCH)/%.o:	%.cc
			@mkdir -p obj/$(ARCH)
			$(CXX) -c $(CXXFLAGS) $< -o $@

all::			lib/$(ARCH)/libPowerSensor.a\
			bin/$(ARCH)/psconfig\
			bin/$(ARCH)/psrun\
			bin/$(ARCH)/pstest\

lib/$(ARCH)/libPowerSensor.a: obj/$(ARCH)/PowerSensor.o
			-mkdir -p lib/$(ARCH)
			$(AR) cr $@ $<

bin/$(ARCH)/psconfig:	obj/$(ARCH)/psconfig.o lib/$(ARCH)/libPowerSensor.a
			-mkdir -p bin/$(ARCH)
			$(CXX) $(CXXFLAGS) obj/$(ARCH)/psconfig.o -Llib/$(ARCH) -lPowerSensor -o $@

bin/$(ARCH)/psrun:	obj/$(ARCH)/psrun.o lib/$(ARCH)/libPowerSensor.a
			-mkdir -p bin/$(ARCH)
			$(CXX) $(CXXFLAGS) obj/$(ARCH)/psrun.o -Llib/$(ARCH) -lPowerSensor -o $@

bin/$(ARCH)/pstest:	obj/$(ARCH)/pstest.o lib/$(ARCH)/libPowerSensor.a
			-mkdir -p bin/$(ARCH)
			$(CXX) $(CXXFLAGS) obj/$(ARCH)/pstest.o -Llib/$(ARCH) -lPowerSensor -o $@

obj/$(ARCH)/psconfig.o:	psconfig.cc PowerSensor.h

obj/$(ARCH)/psrun.o:	psrun.cc PowerSensor.h

obj/$(ARCH)/pstest.o:	pstest.cc PowerSensor.h

obj/$(ARCH)/PowerSensor.o: PowerSensor.cc PowerSensor.h Semaphore.h
