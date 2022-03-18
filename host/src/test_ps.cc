#include <iostream>
#include <iomanip>
#include "PowerSensor.hpp"

#include <unistd.h>

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

  double volt, ampere, watt;

  std::cout << "seconds: " << seconds << std::endl;
  std::cout << "Watt: " << totalWatt << std::endl;
  std::cout << "Joule: " << joules << std::endl;
  std::cout << "Volt\tAmpere\tWatt" << std::endl;
  std::cout << std::setprecision(4) << std::fixed;

  for (ssize_t id=0; id < PowerSensor::MAX_PAIRS; id++) {
    volt = PowerSensor::Volt(firstState, secondState, id);
    ampere = PowerSensor::Ampere(firstState, secondState, id);
    watt = PowerSensor::Watt(firstState, secondState, id);
    std::cout << volt << '\t' << ampere << '\t' << watt << std::endl;
  }

}
