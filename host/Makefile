CXX ?=				g++
CXXFLAGS =			-std=c++11 -O2 -g -pthread -fopenmp
INC = -Iinclude
LIB = -Llib -lPowerSensor

obj/%.o: src/%.cc include/PowerSensor.h
	-mkdir -p obj
	$(CXX) -c $(CXXFLAGS) $(INC) $< -o $@

all:: host bin/test_ps

host:: lib/libPowerSensor.a

lib/libPowerSensor.a: obj/PowerSensor.o
	-mkdir -p lib
	$(AR) rcs $@ $<

bin/test_ps: obj/test_ps.o
	-mkdir -p bin
	$(CXX) $(CXXFLAGS) $(INC) $(LIB) $< -o $@

clean:
	$(RM) -r bin lib obj
