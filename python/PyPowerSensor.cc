#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "PowerSensor.hpp"

namespace py = pybind11;


PYBIND11_MODULE(powersensor, m) {
  m.attr("__version__") = "1.4.1";

  m.attr("MAX_PAIRS") = py::int_(PowerSensor3::MAX_PAIRS);

  m.def("Joules", &PowerSensor3::Joules, "Compute total energy usage between two states",
    py::arg("first_state"), py::arg("second_state"), py::arg("sensor_pair"));
  m.def("seconds", &PowerSensor3::seconds, "Compute time difference between two states",
    py::arg("first_state"), py::arg("second_state"));
  m.def("Watt", &PowerSensor3::Watt, "Compute average power between two states",
    py::arg("first_state"), py::arg("second_state"), py::arg("sensor_pair"));

  py::class_<PowerSensor3::PowerSensor>(m, "PowerSensor")
    .def(py::init<const char*>())
    .def("read", &PowerSensor3::PowerSensor::read, "Read current sensor values")
    .def("dump", &PowerSensor3::PowerSensor::dump,
      "Dump sensor values to file. Set filename to empty string to stop dumping", py::arg("filename"))
    .def("mark", static_cast<void (PowerSensor3::PowerSensor::*)(char)>(&PowerSensor3::PowerSensor::mark),
      "Add given marker character to dump file", py::arg("Marker character"))
    .def("get_name", &PowerSensor3::PowerSensor::getPairName, "Get sensor pair name", py::arg("pair_id"));

  py::class_<PowerSensor3::State>(m, "State")
    .def_readonly("consumed_energy", &PowerSensor3::State::consumedEnergy,
      "Total energy consumption (J), counted from initialization of PowerSensor device")
    .def_readonly("current", &PowerSensor3::State::current, "Current current (A)")
    .def_readonly("voltage", &PowerSensor3::State::voltage, "Current voltage (V)")
    .def_readonly("time_at_read", &PowerSensor3::State::timeAtRead, "Current time (s)");
}
