#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "PowerSensor.hpp"

namespace py = pybind11;


PYBIND11_MODULE(powersensor, m) {
  m.attr("MAX_PAIRS") = py::int_(PowerSensor::MAX_PAIRS);

  m.def("Joules", &PowerSensor::Joules, "Compute total energy usage between two states",
    py::arg("first_state"), py::arg("second_state"), py::arg("sensor_pair"));
  m.def("seconds", &PowerSensor::seconds, "Compute time difference between two states",
    py::arg("first_state"), py::arg("second_state"));
  m.def("Watt", &PowerSensor::Watt, "Compute average power between two states",
    py::arg("first_state"), py::arg("second_state"), py::arg("sensor_pair"));

  py::class_<PowerSensor::PowerSensor>(m, "PowerSensor")
    .def(py::init<const char*>())
    .def("read", &PowerSensor::PowerSensor::read, "Read current sensor values")
    .def("dump", &PowerSensor::PowerSensor::dump,
      "Dump sensor values to file. Set filename to empty string to stop dumping", py::arg("filename"))
    .def("mark", static_cast<void (PowerSensor::PowerSensor::*)(char)>(&PowerSensor::PowerSensor::mark),
      "Add given marker character to dump file", py::arg("Marker character"));

  py::class_<PowerSensor::State>(m, "State")
    .def_readonly("consumed_energy", &PowerSensor::State::consumedEnergy,
      "Total energy consumption (J), counted from initialization of PowerSensor device")
    .def_readonly("current", &PowerSensor::State::current, "Current current (A)")
    .def_readonly("voltage", &PowerSensor::State::voltage, "Current voltage (V)")
    .def_readonly("time_at_read", &PowerSensor::State::timeAtRead, "Current time (s)");
}
