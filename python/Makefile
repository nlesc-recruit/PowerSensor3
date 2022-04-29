CXX ?=              g++
CXXFLAGS =          -std=c++11 -O2 -g -pthread -fopenmp
INC=            -I../host/include/
LIB=                -L../host/lib -lPowerSensor

ifeq ($(OS), Windows_NT)
	os := Windows
else
	os := $(shell uname -s)
endif

# On Mac this flag needs to be added, see https://pybind11.readthedocs.io/en/stable/compiling.html
ifeq ($(os), Darwin)
	CXXFLAGS += -undefined dynamic_lookup
endif

all: python

python: PyPowerSensor.cc
	$(CXX) $(CXXFLAGS) $(INC) $(LIB) -Wall -shared -fPIC $(shell python3 -m pybind11 --includes) -Wl,-rpath,$(shell pwd)/../host/lib PyPowerSensor.cc -o powersensor$(shell python3-config --extension-suffix)

clean:
	$(RM) -r powersensor$(shell python3-config --extension-suffix)*
