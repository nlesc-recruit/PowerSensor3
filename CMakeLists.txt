cmake_minimum_required(VERSION 3.20.1)

project(PowerSensor3 VERSION 1.4.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall")

option(PYTHON_BINDINGS "Generate python bindings." OFF)

include(GNUInstallDirs)
add_subdirectory(host)

if(PYTHON_BINDINGS)
    add_subdirectory(python)
endif()
