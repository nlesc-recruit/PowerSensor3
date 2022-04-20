#include <pybind11/pybind11.h>
#include "PowerSensor.hpp"

namespace py = pybind11;


PYBIND11_MODULE(powersensor, m) {
  py::class_<PowerSensor::PowerSensor>(m, "PowerSensor")
    .def(py::init<const char*>())
    .def("read", &PowerSensor::PowerSensor::read);

  py::class_<PowerSensor::State>(m, "State")
    .def_readonly("consumed_energy", &PowerSensor::State::consumedEnergy);
}
