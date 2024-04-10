set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
find_package(OpenMP REQUIRED)
find_package(pmt REQUIRED)

pybind11_add_module(powersensor PyPowerSensor.cc)

include_directories(../host/include)
target_link_libraries(powersensor PRIVATE PowerSensor OpenMP::OpenMP_CXX pmt)
install(TARGETS powersensor DESTINATION lib/python)
