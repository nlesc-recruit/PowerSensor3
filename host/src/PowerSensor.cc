#include "PowerSensor.hpp"

#include <iostream>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


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

    // set control mode flags;
    terminalOptions.c_cflag |= CLOCAL | CREAD | CS8;

    // set input mode flags;
    terminalOptions.c_iflag = 0;

    // clear local mode flag
    terminalOptions.c_lflag = 0;

    // clear output mode flag;
    terminalOptions.c_oflag = 0;

    // set control characters;
    terminalOptions.c_cc[VMIN] = 0;
    terminalOptions.c_cc[VTIME] = 0;

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

void PowerSensor::readLevelFromDevice(unsigned int &sensorNumber, uint16_t &level) {
    // buffer for one set of sensor data (2 bytes)
    uint8_t buffer[2];
    ssize_t retVal, bytesRead = 0;
    // loop exits when a valid value has been read from the device
    while(true) {
      // read full buffer
      do {
        if ((retVal = ::read(fd, (char*) &buffer + bytesRead, sizeof(buffer) - bytesRead)) < 0) {
          perror("read");
          exit(1);
        }
      } while ((bytesRead += retVal) < sizeof buffer);

      // buffer is full, check the marker bits
      if (((buffer[0] >> 7) == 1) & ((buffer[1] >> 7) == 0)) {
        // marker bits are ok, extract the values
        sensorNumber = (buffer[0] >> 4) & 0x7;
        level = ((buffer[0] & 0xF) << 6) | (buffer[1] & 0x3F);
        return;
      } else {
        // marker bytes are wrong. Assume a byte was dropped: drop first byte and try again
        buffer[0] = buffer[1];
        bytesRead = 1;
      }
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

} // namespace PowerSensor
