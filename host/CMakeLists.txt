set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(PowerSensor src/PowerSensor.cc src/sensors.cc)
add_executable(psconfig src/psconfig.cc)
add_executable(pstest src/pstest.cc)
add_executable(psrun src/psrun.cc)

target_link_libraries(PowerSensor Threads::Threads)
set_property(TARGET PowerSensor PROPERTY POSITION_INDEPENDENT_CODE ON)

include_directories(include/)
target_link_libraries(psconfig PowerSensor)
if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    target_link_libraries(psconfig stdc++fs)
endif()
target_link_libraries(pstest PowerSensor)
target_link_libraries(psrun PowerSensor)

install(TARGETS psconfig pstest psrun DESTINATION bin)
install(FILES include/PowerSensor.hpp include/Semaphore.hpp DESTINATION include)
install(TARGETS PowerSensor DESTINATION lib)
