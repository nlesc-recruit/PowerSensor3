#include <unistd.h>

#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>

#include "PowerSensor.hpp"

int main(int argc, char** argv) {
  std::string device = "/dev/cu.usbmodem386A367F32371";

  if (argc < 2) {
    std::cout << "./test_ps <seconds> [dumpfile]" << std::endl;
    return 1;
  }

  int duration = atoi(argv[1]);
  std::string filename;
  if (argc == 3) {
    filename = argv[2];
  } else {
    filename = "";
  }

  std::cout << "Measuring for " << duration << " s" << std::endl;

  PowerSensor::PowerSensor ps(device);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  PowerSensor::State firstState = ps.read();
  ps.dump(filename.c_str());
  std::this_thread::sleep_for(std::chrono::seconds(duration));
  PowerSensor::State secondState = ps.read();

  // ps.mark(firstState, secondState, "some name", 1);

  ps.dump("");  // stop dumping

  double joules = PowerSensor::Joules(firstState, secondState);
  double totalWatt = PowerSensor::Watt(firstState, secondState);
  double seconds = PowerSensor::seconds(firstState, secondState);

  std::cout << "Totals:" << std::endl;
  std::cout << "seconds: " << seconds << std::endl;
  std::cout << "Watt: " << totalWatt << std::endl;
  std::cout << "Joule: " << joules << std::endl;

  std::cout << std::endl << "Values per sensor pair:" << std::endl;
  std::cout << "Volt\tAmpere\tWatt" << std::endl;
  std::cout << std::setprecision(4) << std::fixed;
  for (ssize_t id=0; id < PowerSensor::MAX_PAIRS; id++) {
    // check current sensor of the given pair
    if (ps.getInUse(2 * id)) {
      double volt = PowerSensor::Volt(firstState, secondState, id);
      double ampere = PowerSensor::Ampere(firstState, secondState, id);
      double watt = PowerSensor::Watt(firstState, secondState, id);
      std::cout << volt << '\t' << ampere << '\t' << watt << std::endl;
    }
  }
}
