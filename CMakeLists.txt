cmake_minimum_required(VERSION 3.22.2)  # ToDo: test with older versions

project(PowerSensor)

option(EMULATOR "PowerSensor emulator to test without connecting to real device." OFF)

add_subdirectory(host)

if(EMULATOR)
    add_subdirectory(emulator)
endif()
