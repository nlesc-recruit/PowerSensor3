find_package(OpenMP REQUIRED)

add_library(PowerSensor src/PowerSensor.cc src/sensors.cc)
add_executable(psconfig src/psconfig.cc)
add_executable(pstest src/pstest.cc)

include_directories(include/)
target_link_libraries(psconfig PowerSensor OpenMP::OpenMP_CXX)
target_link_libraries(pstest PowerSensor OpenMP::OpenMP_CXX)

install(TARGETS psconfig pstest DESTINATION bin)
install(FILES include/PowerSensor.hpp DESTINATION include)
install(TARGETS PowerSensor DESTINATION lib)
