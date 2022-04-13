#include <omp.h>
#include <unistd.h>

#include <iostream>
#include <cstring>

#include "PowerSensor.hpp"

namespace PowerSensor {

void PowerSensor::Sensor::readFromEEPROM(int fd) {
  EEPROM eeprom;
  ssize_t retVal, bytesRead = 0;
  do {
      if ((retVal = ::read(fd, reinterpret_cast<char *>(&eeprom) + bytesRead, sizeof eeprom - bytesRead)) < 0) {
        perror("read device");
        exit(1);
      }
  } while ((bytesRead += retVal) < sizeof eeprom);

  setType(eeprom.type);
  setVref(eeprom.vref);
  setSlope(eeprom.slope);
  setInUse(eeprom.inUse);

  reset();
}

void PowerSensor::Sensor::writeToEEPROM(int fd) const {
  EEPROM eeprom;

  strlcpy(eeprom.type, type, sizeof type);
  eeprom.vref = vref;
  eeprom.slope = slope;
  eeprom.inUse = inUse;

  ssize_t retVal, bytesWritten = 0;
  do {
      if ((retVal = ::write(fd, reinterpret_cast<char *>(&eeprom) + bytesWritten, sizeof eeprom - bytesWritten)) < 0) {
        perror("write device");
        exit(1);
      }
  } while ((bytesWritten += retVal) < sizeof eeprom);
}

void PowerSensor::Sensor::reset() {
  valueAtLastMeasurement = 0;
}

void PowerSensor::Sensor::updateLevel(uint16_t level) {
  this->level = level;
  valueAtLastMeasurement = slope * (VOLTAGE * level / MAX_LEVEL - vref);
}

double PowerSensor::Sensor::getValue() const {
  return valueAtLastMeasurement;
}

void PowerSensor::Sensor::setType(const char* type) {
  strlcpy(this->type, type, sizeof type);
}

void PowerSensor::Sensor::setVref(const float vref) {
  this->vref = vref;
}

void PowerSensor::Sensor::setSlope(const float slope) {
  this->slope = slope;
}

void PowerSensor::Sensor::setInUse(const bool inUse) {
  this->inUse = inUse;
}

}  // namespace PowerSensor
