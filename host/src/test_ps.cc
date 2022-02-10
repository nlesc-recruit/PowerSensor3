#include <iostream>
#include "PowerSensor.hpp"

#include <unistd.h>

int main() {
  const char* device = "/dev/cu.usbmodem207338A658481";

  PowerSensor::PowerSensor ps(device);
  usleep(1000*10);
  for (int i=0; i < PowerSensor::MAX_SENSORS; i++) {
    std::cout << ps.sensors[i].level << std::endl;
  }
}
