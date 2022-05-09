#include <omp.h>
#include <unistd.h>
#include <string.h>

#include <iostream>
#include <cstring>

#include "PowerSensor.hpp"

namespace PowerSensor {

void PowerSensor::Sensor::readFromEEPROM(int fd) {
  EEPROM eeprom;
  unsigned int retVal, bytesRead = 0;
  do {
      if ((retVal = ::read(fd, reinterpret_cast<char *>(&eeprom) + bytesRead, sizeof eeprom - bytesRead)) < 0) {
        perror("read device");
        exit(1);
      }
  } while ((bytesRead += retVal) < sizeof eeprom);

  // If EEPROM is corrupted, type name may be too long. Avoid unrecoverable situation by changing type if that happens
  std::string type = eeprom.type;
  if (type.length() >= MAX_TYPE_LENGTH) {
    std::cerr << "Read invalid type from device, EEPROM data may be corrupt" << std::endl;
    type = "INVALID";
  }

  setType(type);
  setVref(eeprom.vref);
  setSensitivity(eeprom.sensitivity);
  setInUse(eeprom.inUse);

  reset();
}

void PowerSensor::Sensor::writeToEEPROM(int fd) const {
  EEPROM eeprom;

  strncpy(eeprom.type, type.c_str(), type.length() + 1);  // plus one for null termination character
  eeprom.vref = vref;
  eeprom.sensitivity = sensitivity;
  eeprom.inUse = inUse;

  unsigned int retVal, bytesWritten = 0;
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
  // sensitivity is converted from mV/(A|V) to (A|V) / V
  valueAtLastMeasurement = (1000. / sensitivity) * (VOLTAGE * level / MAX_LEVEL - vref);
}

double PowerSensor::Sensor::getValue() const {
  return valueAtLastMeasurement;
}

void PowerSensor::Sensor::setType(const std::string type) {
  if (type.length() >= MAX_TYPE_LENGTH) {
    // MAX_TYPE_LENGTH includes null termination character
    std::cerr << "Sensor type name can be at most " << MAX_TYPE_LENGTH - 1 << " characters" << std::endl;
    exit(1);
  } else {
    this->type = type;
  }
}

void PowerSensor::Sensor::setVref(const float vref) {
  this->vref = vref;
}

void PowerSensor::Sensor::setSensitivity(const float sensitivity) {
  this->sensitivity = sensitivity;
}

void PowerSensor::Sensor::setInUse(const bool inUse) {
  this->inUse = inUse;
}

}  // namespace PowerSensor
