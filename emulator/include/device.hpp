#pragma once

#include "PowerSensor.hpp"

namespace PowerSensorEmulator {

struct EEPROM {
  struct {
    char type[16];
    float vref;
    float sensitivity;
    bool inUse;
  } __attribute__((packed)) sensors[PowerSensor::MAX_SENSORS];
};

void serialEventLoop(int fd);
void writeLoop(int fd);

void initEEPROM();
void setSensorData();
void readConfig(int fd);
void writeConfig(int fd);
void stop();

}  //  namespace PowerSensorEmulator
