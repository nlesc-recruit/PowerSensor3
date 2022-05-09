cmake_minimum_required(VERSION 3.22.2)  # ToDo: test with older versions

project(PowerSensor)

set(CMAKE_CXX_STANDARD 11)

option(EMULATOR "PowerSensor emulator to test without connecting to real device." OFF)
option(PYTHON_BINDINGS "Generate python bindings." OFF)

add_subdirectory(host)

if(EMULATOR)
    add_subdirectory(emulator)
endif()

if(PYTHON_BINDINGS)
    add_subdirectory(python)
endif()
