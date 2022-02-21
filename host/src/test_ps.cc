#include <iostream>
#include "PowerSensor.hpp"

#include <unistd.h>

int main() {
  const char* device = "/dev/cu.usbmodem207338A658481";

  PowerSensor::PowerSensor ps(device);

  PowerSensor::State firstState = ps.read();
  usleep(1000 * 10);  // 10 ms
  PowerSensor::State secondState = ps.read();

  double joules = PowerSensor::Joules(firstState, secondState);
  double watt = PowerSensor::Watt(firstState, secondState);
  double seconds = PowerSensor::seconds(firstState, secondState);

  std::cout << "seconds: " << seconds << std::endl;
  std::cout << "Watt: " << watt << std::endl;
  std::cout << "Joule: " << joules << std::endl;

  ps.dump("dumpfile.txt");
  usleep(1000 * 10);  // 10 ms
}
