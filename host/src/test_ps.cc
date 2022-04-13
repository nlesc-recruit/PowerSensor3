#include <unistd.h>

#include <iostream>
#include <iomanip>

#include "PowerSensor.hpp"

int main() {
  const char* device = "/dev/cu.usbmodem386A367F32371";

  PowerSensor::PowerSensor ps(device);
  usleep(1000*10);  // 10 ms

  PowerSensor::State firstState = ps.read();
  ps.dump("dumpfile.txt");
  usleep(1000 * 5);
  ps.mark('A');
  usleep(1000 * 10);
  PowerSensor::State secondState = ps.read();

  ps.mark(firstState, secondState, "some name", 1);

  ps.dump("");

  double joules = PowerSensor::Joules(firstState, secondState);
  double totalWatt = PowerSensor::Watt(firstState, secondState);
  double seconds = PowerSensor::seconds(firstState, secondState);

  std::cout << "seconds: " << seconds << std::endl;
  std::cout << "Watt: " << totalWatt << std::endl;
  std::cout << "Joule: " << joules << std::endl;
  std::cout << "Volt\tAmpere\tWatt" << std::endl;
  std::cout << std::setprecision(4) << std::fixed;

  for (ssize_t id=0; id < PowerSensor::MAX_PAIRS; id++) {
    double volt = PowerSensor::Volt(firstState, secondState, id);
    double ampere = PowerSensor::Ampere(firstState, secondState, id);
    double watt = PowerSensor::Watt(firstState, secondState, id);
    std::cout << volt << '\t' << ampere << '\t' << watt << std::endl;
  }
}
