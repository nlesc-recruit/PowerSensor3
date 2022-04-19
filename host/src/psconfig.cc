#include <unistd.h>

#include <cstring>
#include <iostream>
#include <memory>

#include "PowerSensor.hpp"


std::unique_ptr<PowerSensor::PowerSensor> powerSensor;
unsigned int sensor = 0;


unsigned int selectSensor(unsigned int sensor) {
  if (sensor >= PowerSensor::MAX_SENSORS) {
    std::cerr << "Invalid sensor ID: " << sensor << ", max value is " << PowerSensor::MAX_SENSORS - 1 << std::endl;
  }
  return sensor;
}


PowerSensor::PowerSensor *getPowerSensor(std::string device) {
  if (device.empty())
    device = "/dev/ttyACM0";
  if (powerSensor.get() == nullptr)
    powerSensor = std::unique_ptr<PowerSensor::PowerSensor>(new PowerSensor::PowerSensor(device.c_str()));

  return powerSensor.get();
}


void measureSensors(PowerSensor::State* startState, PowerSensor::State* stopState) {
  *startState = powerSensor->read();
  sleep(2);
  *stopState = powerSensor->read();
}


void print() {
  PowerSensor::State startState, stopState;

  measureSensors(&startState, &stopState);

  char type[16];
  std::string sensorType;
  char unit;
  for (unsigned sensor = 0; sensor < PowerSensor::MAX_SENSORS; sensor++) {
    powerSensor->getType(sensor, type);

    if (sensor % 2 == 0) {
      sensorType = "current";
      unit = 'A';
    } else {
      sensorType = "voltage";
      unit = 'V';
    }

    std::cout << "sensor " << sensor << " (" << sensorType << "): "
      "type: " << type << ", "
      "Vref: " << powerSensor->getVref(sensor) << " V, "
      "Sensitivity: " << powerSensor->getSensitivity(sensor) << " " << unit << " / V, "
      "Status: " << (powerSensor->getInUse(sensor) ? "on" : "off") << std::endl;
  }

  double totalUsage = 0;
  for (unsigned int pair = 0; pair < PowerSensor::MAX_PAIRS; pair++) {
      double usage = Watt(startState, stopState, pair);
      totalUsage += usage;
      std::cout << "Current usage pair " << pair << ": " << usage << " W" << std::endl;
  }
  std::cout << "Total usage: " << totalUsage << " W" << std::endl;
}


void usage(char *argv[]) {
  std::cerr << "usage: " << argv[0] << " [-h] [-d device] [-s sensor] [-t type] "
    "[-v volt] [-a | -n nullLevel] [-o] [-p]" << std::endl;
  std::cerr << "-h prints this help" << std::endl;
  std::cerr << "-d selects the device (default: /dev/ttyACM0)" << std::endl;
  std::cerr << "-s selects the sensor (0-" << PowerSensor::MAX_SENSORS << ")" << std::endl;
  std::cerr << "-t sets the sensor type, (valid types TBD)" << std::endl;
  std::cerr << "-v sets the reference voltage level" << std::endl;
  std::cerr << "-n set the sensitivity" << std::endl;
  std::cerr << "-o turns a sensor on (1) or off (0)" << std::endl;
  std::cerr << "-p prints configured values" << std::endl;
  std::cerr << "example: " << argv[0] << " -d /dev/ttyACM0 -s 0 -t ACS712-20 -v 1.65 "
    "-n 1.0 -o 1 -s 1 -tACS712-5 -v1.7 -p" << std::endl;
  exit(1);
}


int main(int argc, char *argv[]) {
  std::string device;
  for (int opt; (opt = getopt(argc, argv, "d:s:i:t:v:n:o:ph")) >= 0;) {
    switch (opt) {
      // device select
      case 'd':
        device = optarg;
        break;

      // sensor select
      case 's':
        sensor = selectSensor((unsigned) atoi(optarg));
        break;

      // sensor type
      case 't':
        getPowerSensor(device)->setType(sensor, optarg);
        break;

      // sensor reference voltage
      case 'v':
        getPowerSensor(device)->setVref(sensor, atof(optarg));
        break;

      // sensor sensitivity
      case 'n':
        getPowerSensor(device)->setSensitivity(sensor, atof(optarg));
        break;

      // sensor on/off
      case 'o':
        getPowerSensor(device)->setInUse(sensor, static_cast<bool>(optarg));
        break;

      // print
      case 'p':
        getPowerSensor(device);
        print();
        break;

      // help
      case 'h':
        usage(argv);
        break;

      default:
        usage(argv);
        break;
    }
  }

  if ((optind < argc) || (argc < 2))
    usage(argv);

  return 0;
}
