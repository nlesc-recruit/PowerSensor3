set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
find_package(OpenMP REQUIRED)

pybind11_add_module(powersensor PyPowerSensor.cc)

include_directories(../host/include)
target_link_libraries(powersensor PRIVATE PowerSensor OpenMP::OpenMP_CXX)

set(PYTHON_SITE_PACKAGES
    "${CMAKE_INSTALL_LIBDIR}/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/powersensor"
)

install(TARGETS powersensor DESTINATION "${PYTHON_SITE_PACKAGES}")
install(FILES __init__.py DESTINATION "${PYTHON_SITE_PACKAGES}")
