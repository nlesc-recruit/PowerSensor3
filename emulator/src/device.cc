#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <iostream>
#include <bitset>
#include <chrono>

#include "device.hpp"

namespace PowerSensorEmulator {

EEPROM eeprom = EEPROM();
uint8_t sensorData[PowerSensor::MAX_SENSORS*2];  // 2 bytes per sensor
bool running = true;
bool streamValues = false;
bool sendMarkerNext = false;
bool sendSingleValue = false;
unsigned int counter = 0;

void initEEPROM() {
  for (ssize_t i = 0; i < PowerSensor::MAX_SENSORS; i++) {
    const char* type = "Type";
    strlcpy(eeprom.sensors[i].type, type, sizeof type);
    eeprom.sensors[i].vref = 1.65;
    eeprom.sensors[i].sensitivity = 1.0;
    eeprom.sensors[i].inUse = true;
  }
}

void setSensorData() {
  static const uint16_t sensorLevels[] = {1023, 877, 731, 585, 438, 292, 146, 0};
  for (ssize_t sensorID = 0; sensorID < PowerSensor::MAX_SENSORS; sensorID++) {
    sensorData[2*sensorID] = ((sensorID & 0x7) << 4) | ((sensorLevels[sensorID] & 0x3C0) >> 6) | (1 << 7);
    sensorData[2*sensorID + 1] = ((sendMarkerNext << 6) | (sensorLevels[sensorID] & 0x3F)) & ~(1 << 7);
    counter++;
    sendMarkerNext = false;
  }
}

void readConfig(int fd) {
  // send EEPROM contents to host
  if (write(fd, &eeprom, sizeof eeprom) != (sizeof eeprom)) {
    perror("Failed to write EEPROM to host");
    exit(1);
  }
}

void writeConfig(int fd) {
  // receive EEPROM contents from host
  unsigned int bytesReceived = 0;
  unsigned int retVal;
  do {
    retVal = read(fd, reinterpret_cast<char*>(&eeprom) + bytesReceived, sizeof(eeprom) - bytesReceived);
    if (retVal < 0) {
      perror("Failed to receive EEPROM from host");
      exit(1);
    }
  } while ((bytesReceived += retVal) < sizeof eeprom);

  // signal to host that data were received
  write(fd, "D", 1);
}

void serialEventLoop(int fd) {
  // ensure the EEPROM is initialized with values
  initEEPROM();

  char command;
  int nbyteRead;
  while (running) {
    nbyteRead = read(fd, &command, sizeof command);
    if (nbyteRead <= 0)
      continue;

    switch (command) {
      // ignore LF
      case 0b1010:
        break;
      std::cerr << "Received command " << command << std::endl;
      case 'A':
        stop();
        break;
      case 'R':
        readConfig(fd);
        break;
      case 'W':
        writeConfig(fd);
        break;
      case 'M':
        sendMarkerNext = true;
        break;
      case 'I':
        sendSingleValue = true;
        break;
      case 'S':
        counter = 0;
        streamValues = true;
        break;
      case 'T':
        streamValues = false;
        break;
      case 'X': {
        uint8_t buffer[] = {0xFF, 0x3F};
        write(fd, buffer, sizeof buffer);
        write(fd, buffer, sizeof buffer);
        break;
      }
      case 'Q':
        write(fd, &counter, sizeof counter);
        break;
      default: {
        std::cerr << "Ignoring unknown command: " << command << " (" << std::bitset<8>(command) << ")." << std::endl;
        break;
      }
    }
  }
}

void writeLoop(int fd) {
  while (running) {
    while (!(streamValues | sendSingleValue)) {
      usleep(1);
      if (!running)
        return;
    }
    setSensorData();
    write(fd, sensorData, sizeof sensorData);
    sendSingleValue = false;
    // sleep for a bit to roughly match real data rate
    std::this_thread::sleep_for(std::chrono::microseconds(15));
  }
}

void stop() {
  running = false;
}

}  //  namespace PowerSensorEmulator
