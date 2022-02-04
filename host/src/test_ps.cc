#include <iostream>
#include "PowerSensor.hpp"

#include <unistd.h>

int main() {
  const char* device = "/dev/cu.usbmodem207338A658481";

  PowerSensor::PowerSensor ps(device);

  unsigned int id;
  uint16_t level;
  // manually enable streaming
  write(ps.fd, "S", 1);
  id = 0;
  while (id != 5) {
    ps.readLevelFromDevice(id, level);
  }
  // manually stop streaming
  write(ps.fd, "T", 1);
  std::cout << id << " " << level << std::endl;
}
