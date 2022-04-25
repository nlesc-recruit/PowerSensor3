#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "PowerSensor.hpp"

namespace py = pybind11;


PYBIND11_MODULE(powersensor, m) {
  m.def("Joules", &PowerSensor::Joules);
  m.def("seconds", &PowerSensor::seconds);
  m.def("Watt", &PowerSensor::Watt);
  m.def("Volt", &PowerSensor::Volt);
  m.def("Ampere", &PowerSensor::Ampere);

  py::class_<PowerSensor::PowerSensor>(m, "PowerSensor")
    .def(py::init<const char*>())
    .def("read", &PowerSensor::PowerSensor::read)
    .def("dump", &PowerSensor::PowerSensor::dump, "Dump sensor values to file", py::arg("filename"))
    .def("mark", static_cast<void (PowerSensor::PowerSensor::*)(char)>(&PowerSensor::PowerSensor::mark));

  py::class_<PowerSensor::State>(m, "State")
    .def_readonly("consumed_energy", &PowerSensor::State::consumedEnergy)
    .def_readonly("current", &PowerSensor::State::current)
    .def_readonly("voltage", &PowerSensor::State::voltage)
    .def_readonly("time_at_read", &PowerSensor::State::timeAtRead);
}
