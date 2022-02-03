#include "PowerSensor.h"

#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <cstring>

// 4M baudrate is not defined by default on Mac
#ifdef __APPLE__
#define B4000000 0010017
#endif

namespace PowerSensor {

  PowerSensor::PowerSensor(const char* device):
    fd(openDevice(device)) {
      readSensorsFromEEPROM();
    };

  PowerSensor::~PowerSensor() {close(fd);};

  int PowerSensor::openDevice(const char* device) {
    int fileDescriptor;

    // opens the file specified by pathname;
    if ((fileDescriptor = open(device, O_RDWR)) < 0)
    {
      perror("open device");
      exit(1);
    }
    // block if an incompatible lock is held by another process;
    if (flock(fileDescriptor, LOCK_EX) < 0)
    {
      perror("flock");
      exit(1);
    }

    // struct for configuring the port for communication with stm32;
    struct termios terminalOptions;

    // gets the current options for the port;
    tcgetattr(fileDescriptor, &terminalOptions);

    // sets the input baud rate;
    cfsetispeed(&terminalOptions, B4000000);

    // sets the output baud rate;
    cfsetospeed(&terminalOptions, B4000000);

    // set control mode flags;
    terminalOptions.c_cflag |= CLOCAL | CREAD | CS8;

    // set input mode flags;
    terminalOptions.c_iflag = 0;

    // clear local mode flag
    terminalOptions.c_lflag = 0;

    // clear output mode flag;
    terminalOptions.c_oflag = 0;

    // set control characters;
    terminalOptions.c_cc[VMIN] = 2;
    terminalOptions.c_cc[VTIME] = 1;

    // commit the options;
    tcsetattr(fileDescriptor, TCSANOW, &terminalOptions);

    // flush anything already in the serial buffer;
    tcflush(fileDescriptor, TCIFLUSH);

    return fileDescriptor;
  }

  void PowerSensor::readSensorsFromEEPROM() {
    if (write(fd, "R", 1) != 1) {
      perror("write device");
      exit(1);
    }
    for (Sensor& sensor: sensors) {
      sensor.readFromEEPROM(fd);
    }
  }

  void PowerSensor::writeSensorsToEEPROM() {
    if (write(fd, "W", 1) != 1) {
      perror("Write device");
      exit(1);
    }
    for (const Sensor& sensor: sensors) {
      sensor.writeToEEPROM(fd);
    }
  }

  void PowerSensor::getType(unsigned int sensorID, char* type) const {
    strncpy(type, sensors[sensorID].type, sizeof sensors[sensorID].type);
  }

  float PowerSensor::getVref(unsigned int sensorID) const {
    return sensors[sensorID].vref;
  }

  float PowerSensor::getSlope(unsigned int sensorID) const {
    return sensors[sensorID].slope;
  }

  uint8_t PowerSensor::getPairId(unsigned int sensorID) const {
    return sensors[sensorID].pairId;
  }

  bool PowerSensor::getInUse(unsigned int sensorID) const {
    return sensors[sensorID].inUse;
  }

  void PowerSensor::setType(unsigned int sensorID, const char* type) {
    sensors[sensorID].setType(type);
  }

  void PowerSensor::setVref(unsigned int sensorID, const float vref) {
    sensors[sensorID].setVref(vref);
  }

  void PowerSensor::setSlope(unsigned int sensorID, const float slope) {
    sensors[sensorID].setSlope(slope);
  }

  void PowerSensor::setPairId(unsigned int sensorID, const uint8_t pairId) {
    sensors[sensorID].setPairId(pairId);
  }

  void PowerSensor::setInUse(unsigned int sensorID, const bool inUse) {
    sensors[sensorID].setInUse(inUse);
  }

  void PowerSensor::Sensor::readFromEEPROM(int fd) {
    EEPROM eeprom;
    ssize_t retVal, bytesRead = 0;
    do {
        if ((retVal = ::read(fd, (char *) &eeprom + bytesRead, sizeof eeprom - bytesRead)) < 0) {
          perror("read device");
          exit(1);
        }
    } while ((bytesRead += retVal) < sizeof eeprom);

    setType(eeprom.type);
    setVref(eeprom.vref);
    setSlope(eeprom.slope);
    setPairId(eeprom.pairId);
    setInUse(eeprom.inUse);
  }

  void PowerSensor::Sensor::writeToEEPROM(int fd) const {
    EEPROM eeprom;

    strncpy(eeprom.type, type, sizeof type);
    eeprom.vref = vref;
    eeprom.slope = slope;
    eeprom.pairId = pairId;
    eeprom.inUse = inUse;

    ssize_t retVal, bytesWritten = 0;
    do {
        if ((retVal = ::write(fd, (char *) &eeprom + bytesWritten, sizeof eeprom - bytesWritten)) < 0) {
          perror("write device");
          exit(1);
        }
    } while ((bytesWritten += retVal) < sizeof eeprom);
  }

  void PowerSensor::Sensor::setType(const char* type) {
    strncpy(this->type, type, sizeof type);
  }

  void PowerSensor::Sensor::setVref(const float vref) {
    this->vref = vref;
  }

  void PowerSensor::Sensor::setSlope(const float slope) {
    this->slope = slope;
  }

  void PowerSensor::Sensor::setPairId(const uint8_t pairId) {
    this->pairId = pairId;
  }

  void PowerSensor::Sensor::setInUse(const bool inUse) {
    this->inUse = inUse;
  }

} // namespace PowerSensor
