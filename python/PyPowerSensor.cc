#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "PowerSensor.hpp"

namespace py = pybind11;


PYBIND11_MODULE(powersensor, m) {
  m.def("Joules", &PowerSensor::Joules, "Compute energy usage between two states", py::arg("first_state"), py::arg("second_state"), py::arg("sensor_pair"));
  m.def("seconds", &PowerSensor::seconds, "Compute time difference between two states", py::arg("first_state"), py::arg("second_state"));
  m.def("Watt", &PowerSensor::Watt, "Compute average power between two states", py::arg("first_state"), py::arg("second_state"), py::arg("sensor_pair"));
  m.def("Volt", &PowerSensor::Volt, "Compute average voltage between two states", py::arg("first_state"), py::arg("second_state"), py::arg("sensor_pair"));
  m.def("Ampere", &PowerSensor::Ampere, "Compute average current between two states", py::arg("first_state"), py::arg("second_state"), py::arg("sensor_pair"));

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
