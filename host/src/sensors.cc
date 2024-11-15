#include <unistd.h>
#include <string.h>

#include <iostream>
#include <cstring>

#include "PowerSensor.hpp"

namespace PowerSensor3 {

/**
 * @brief Read configuration of single sensor from device EEPROM
 *
 * @param fd file descriptor to device
 */
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
    std::cerr << "Read invalid sensor type from device, EEPROM data may be corrupt" << std::endl;
    type = "INVALID";
  }

  // Same check as for type name
  std::string pairName = eeprom.pairName;
  if (pairName.length() >= MAX_PAIRNAME_LENGTH) {
    std::cerr << "Read invalid sensor pair name from device, EEPROM data may be corrupt" << std::endl;
    pairName = "INVALID";
  }

  setType(type);
  setPairName(pairName);
  setVref(eeprom.vref);
  setSensitivity(eeprom.sensitivity);
  setInUse(eeprom.inUse);

  reset();
}

/**
 * @brief Write configuration of single sensor to device EEPROM
 *
 * @param fd file descriptor to device
 */
void PowerSensor::Sensor::writeToEEPROM(int fd) const {
  EEPROM eeprom;

  strncpy(eeprom.type, type.c_str(), type.length() + 1);  // plus one for null termination character
  strncpy(eeprom.pairName, pairName.c_str(), pairName.length() + 1);  // plus one for null termination character
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

/**
 * @brief Reset the sensor value to 0
 *
 */
void PowerSensor::Sensor::reset() {
  valueAtLastMeasurement = 0;
}

/**
 * @brief Update calibrated sensor value based on raw level
 *
 * @param level
 */
void PowerSensor::Sensor::updateLevel(uint16_t level) {
  this->level = level;
  valueAtLastMeasurement = (VOLTAGE * level / MAX_LEVEL - vref) / sensitivity;
}

/**
 * @brief Get current calibrated sensor value
 *
 * @return double
 */
double PowerSensor::Sensor::getValue() const {
  return valueAtLastMeasurement;
}

/**
 * @brief Set type of sensor
 *
 * @param type
 */
void PowerSensor::Sensor::setType(const std::string type) {
  if (type.length() >= MAX_TYPE_LENGTH) {
    // MAX_TYPE_LENGTH includes null termination character
    std::cerr << "Sensor type name can be at most " << MAX_TYPE_LENGTH - 1 << " characters" << std::endl;
    exit(1);
  } else {
    this->type = type;
  }
}

/**
 * @brief Set name of sensor pair
 *
 * @param pairName
 */
void PowerSensor::Sensor::setPairName(const std::string pairName) {
  if (pairName.length() >= MAX_PAIRNAME_LENGTH) {
    // MAX_PAIRNAME_LENGTH includes null termination character
    std::cerr << "Sensor pair name can be at most " << MAX_PAIRNAME_LENGTH - 1 << " characters" << std::endl;
    exit(1);
  } else {
    this->pairName = pairName;
  }
}


/**
 * @brief Set reference voltage of sensor
 *
 * @param vref
 */
void PowerSensor::Sensor::setVref(const float vref) {
  this->vref = vref;
}

/**
 * @brief Set sensitivity of sensor
 *
 * @param sensitivity
 */
void PowerSensor::Sensor::setSensitivity(const float sensitivity) {
  this->sensitivity = sensitivity;
}

/**
 * @brief Set whether or not the sensor is in use
 *
 * @param inUse
 */
void PowerSensor::Sensor::setInUse(const bool inUse) {
  this->inUse = inUse;
}

/**
 * @brief Set polarity of sensor
 *
 * @param polarity (-1 or 1)
 */
void PowerSensor::Sensor::setPolarity(const int polarity) {
  if (polarity == 1) {
    this->sensitivity = std::abs(this->sensitivity);
  } else if (polarity == -1) {
    this->sensitivity = -std::abs(this->sensitivity);
  } else {
    std::cerr << "Polarity must be -1 or 1, got " << polarity << std::endl;
    exit(1);
  }
}

}  // namespace PowerSensor3
