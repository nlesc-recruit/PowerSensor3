cmake_minimum_required(VERSION 3.20.1)

project(PowerSensor3 VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall")

option(EMULATOR "PowerSensor emulator to test without connecting to real device." OFF)
option(PYTHON_BINDINGS "Generate python bindings." OFF)

add_subdirectory(host)

if(EMULATOR)
    add_subdirectory(emulator)
endif()

if(PYTHON_BINDINGS)
    add_subdirectory(python)
endif()
