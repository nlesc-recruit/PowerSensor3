find_package(OpenMP REQUIRED)

add_library(PowerSensor src/PowerSensor.cc src/sensors.cc)
add_executable(psconfig src/psconfig.cc)
add_executable(pstest src/pstest.cc)
add_executable(psrun src/psrun.cc)

set_property(TARGET PowerSensor PROPERTY POSITION_INDEPENDENT_CODE ON)

find_package(pmt REQUIRED)
include_directories(include/)
target_link_libraries(psconfig PowerSensor OpenMP::OpenMP_CXX pmt)
if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    target_link_libraries(psconfig stdc++fs)
endif()
target_link_libraries(pstest PowerSensor OpenMP::OpenMP_CXX pmt)
target_link_libraries(psrun PowerSensor OpenMP::OpenMP_CXX pmt)

install(TARGETS psconfig pstest psrun DESTINATION bin)
install(FILES include/PowerSensor.hpp include/Semaphore.hpp DESTINATION include)
install(TARGETS PowerSensor DESTINATION lib)
