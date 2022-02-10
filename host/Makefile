CXX ?=				g++
CXXFLAGS =			-std=c++11 -O2 -g -pthread -fopenmp
INC = -Iinclude
LIB = -Llib -lPowerSensor

obj/%.o: src/%.cc
	-mkdir -p obj
	$(CXX) -c $(CXXFLAGS) $(INC) $< -o $@

all:: host bin

host:: lib/libPowerSensor.a

bin:: bin/test_ps

lib/libPowerSensor.a: obj/PowerSensor.o obj/sensors.o
	-mkdir -p lib
	$(AR) rcs $@ $^

bin/test_ps: obj/test_ps.o
	-mkdir -p bin
	$(CXX) $(CXXFLAGS) $(INC) $(LIB) $< -o $@

clean:
	$(RM) -r bin lib obj
