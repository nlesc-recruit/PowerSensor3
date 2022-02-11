#include <iostream>
#include "PowerSensor.hpp"

#include <unistd.h>

int main() {
  const char* device = "/dev/cu.usbmodem207338A658481";

  PowerSensor::PowerSensor ps(device);
  ps.dump("dumpfile.txt");
  usleep(1000*10);
}
