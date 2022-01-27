#include <iostream>
#include "PowerSensor.h"

int main() {
  const char* device = "/dev/cu.usbmodem207338A658481";

  PowerSensor::PowerSensor ps = PowerSensor::PowerSensor(device);
}

