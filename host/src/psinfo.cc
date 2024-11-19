#include <unistd.h>

#include <iostream>
#include <chrono>

#include "PowerSensor.hpp"


std::unique_ptr<PowerSensor3::PowerSensor> powerSensor;


void getPowerSensor(std::string device) {
  if (device.empty())
    device = "/dev/ttyACM0";
  if (powerSensor.get() == nullptr)
    powerSensor = std::unique_ptr<PowerSensor3::PowerSensor>(new PowerSensor3::PowerSensor(device));
}


void printInfo() {
  std::cout << "Device firmware version: " << powerSensor->getVersion() << std::endl;

  PowerSensor3::State state = powerSensor->read();

  double totalPower = 0;

  for (unsigned pair = 0; pair < PowerSensor3::MAX_PAIRS; pair++) {
    const std::string pairName = powerSensor->getPairName(pair);
    const unsigned sensor_current = 2 * pair;
    const unsigned sensor_voltage = 2 * pair + 1;
    // only print detailed info for active sensor pairs
    if (powerSensor->getInUse(sensor_current) && powerSensor->getInUse(sensor_voltage)) {
        totalPower += state.current[pair] * state.voltage[pair];
        std::cout << "Sensor pair " << pair << ":" << std::endl;
        std::cout << "\tName: " << pairName << std::endl;
        std::cout << "\tLatest measured power: " << state.current[pair] * state.voltage[pair] << " W" << std::endl;
        std::cout << "\tCurrent sensor details:" << std::endl;
	      std::cout << "\t\tSensor: " << sensor_current << std::endl;
     	std::cout << "\t\tConfiguration:" << std::endl;
        std::cout << "\t\t\tVref: " << powerSensor->getVref(sensor_current) << " V" << std::endl;
        std::cout << "\t\t\tSensitivity: " << 1000 * powerSensor->getSensitivity(sensor_current) << " mV/A" << std::endl;
        std::cout << "\t\t\tPolarity: " << powerSensor->getPolarity(sensor_current) << std::endl;
        std::cout << "\t\tLatest value: " << state.current[pair] << " A" << std::endl;
        std::cout << "\tVoltage sensor details:" << std::endl;
        std::cout << "\t\tSensor: " << sensor_voltage << std::endl;
        std::cout << "\t\tConfiguration:" << std::endl;
        std::cout << "\t\t\tVref: " << powerSensor->getVref(sensor_voltage) << " V" << std::endl;
        std::cout << "\t\t\tGain: " << powerSensor->getSensitivity(sensor_voltage) << std::endl;
        std::cout << "\t\t\tPolarity: " << powerSensor->getPolarity(sensor_voltage) << std::endl;
        std::cout << "\t\tLatest value: " << state.voltage[pair] << " V" << std::endl;
    } else {
        std::cout << "Sensor pair " << pair << ": inactive" << std::endl;
    }
  }
  std::cout << std::endl << "Total power: " << totalPower << " W" << std::endl;
}


void usage(char *argv[]) {
  std::cerr << "usage: " << argv[0] << " [-h] [-d device]" << std::endl;
  std::cerr << "-h prints this help" << std::endl;
  std::cerr << "-d selects the device (default: /dev/ttyACM0)" << std::endl;
  exit(1);
}


int main(int argc, char *argv[]) {
  std::string device;

  std::cout << "psinfo version " <<  PowerSensor3::POWERSENSOR_VERSION << std::endl << std::endl;
  for (int opt; (opt = getopt(argc, argv, "d:h")) >= 0;) {
    switch (opt) {
      // device select
      case 'd':
        device = optarg;
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

  if ((optind < argc))
    usage(argv);

  getPowerSensor(device);
  printInfo();

  return 0;
}
