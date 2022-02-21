CXX ?=				g++
CXXFLAGS =			-std=c++11 -O2 -g -pthread -fopenmp -fPIC
INC = -Ihost/include
LIB = -Lhost/lib -lPowerSensor

host/obj/%.o: host/src/%.cc
	-mkdir -p host/obj
	$(CXX) -c $(CXXFLAGS) $(INC) $< -o $@

all:: host bin lib

host:: lib

bin:: host/bin/test_ps

lib: host/obj/PowerSensor.o host/obj/sensors.o
	-mkdir -p host/lib
	$(AR) rcs host/lib/libPowerSensor.a $^
	$(CXX) -shared -fopenmp -o host/lib/libPowerSensor.so $^

host/bin/%: host/obj/%.o
	-mkdir -p host/bin
	$(CXX) $(CXXFLAGS) $(INC) $(LIB) $< -o $@

clean:
	$(RM) -r host/bin host/lib host/obj
