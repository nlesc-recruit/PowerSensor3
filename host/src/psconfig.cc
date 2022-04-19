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

float getDefaultSensitivity(std::string type) {
  // Sensitivity is given in V/A for current sensors, or V/V for voltage sensors
  float sensitivity = 0;

  // Current sensors. These are of type MLX91221KDF-ABF-0NN-RE, where NN is the number after
  // MLX in the shortened name
  if (type.compare("MLX10") == 0) {
    sensitivity = .120;
  } else if (type.compare("MLX20") == 0) {
    sensitivity = .0625;
  } else if (type.compare("MLX50") == 0) {
    sensitivity = .025;
  } else if (type.compare("MLX75") == 0) {
    sensitivity = .01667;
  } else {
    std::cerr << "No sensitivity known for sensor of type " << type << "."
                 " Please make sure to set sensitivity manually with the -n option." << std::endl;
  }
  return sensitivity;
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
    "[-a | -v volt] [-n sensitivity] [-o on/off] [-p]" << std::endl;
  std::cerr << "-h prints this help" << std::endl;
  std::cerr << "-d selects the device (default: /dev/ttyACM0)" << std::endl;
  std::cerr << "-s selects the sensor (0-" << PowerSensor::MAX_SENSORS << ")" << std::endl;
  std::cerr << "-t sets the sensor type. This also sets the sensitivity to the default value if "
               "the sensor is of a type known to this programme (see list at the bottom of this help)." << std::endl;
  std::cerr << "-v sets the reference voltage level" << std::endl;
  std::cerr << "-n set the sensitivity (in V/A for current sensors, or V/V for voltage sensors)" << std::endl;
  std::cerr << "-o turns a sensor on (1) or off (0)" << std::endl;
  std::cerr << "-p prints configured values" << std::endl;
  std::cerr << "example: " << argv[0] << " -d /dev/ttyACM0 -s 0 -t MLX10 -v 1.65 "
               "-n .015 -o 1 -s 1 -tMLX20 -v1.7 -p" << std::endl;
  std::cerr << "Known sensor types: MLX10, MLX20, MLX50, MLX75." << std::endl;
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

      // sensor type, also sets default sensitivity
      case 't': {
        getPowerSensor(device)->setType(sensor, optarg);
        // set default sensitivity (zero for unknown sensor)
        float sensitivity = getDefaultSensitivity(optarg);
        std::cerr << "Would set sensitivity to " << sensitivity << std::endl;
        // if (sensitivity > 0)
          // getPowerSensor(device)->setSensitivity(sensor, sensitivity);
        break;
      }

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
