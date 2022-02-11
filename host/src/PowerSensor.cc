#include "PowerSensor.hpp"

#include <iostream>
#include <cstring>

#include <fcntl.h>
#include <omp.h>
#include <termios.h>
#include <unistd.h>


namespace PowerSensor {

  PowerSensor::PowerSensor(const char* device):
    fd(openDevice(device)),
    thread(nullptr),
    startTime(omp_get_wtime())
    {
      readSensorsFromEEPROM();
      startIOThread();
    };

  PowerSensor::~PowerSensor() {
    stopIOThread();

    if (close(fd)) {
      perror("close device");
    }
  }

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

bool PowerSensor::readLevelFromDevice(unsigned int &sensorNumber, uint16_t &level) {
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

      // buffer is full, check if stop was received
      if (buffer[0] == 0xFF && buffer[1] == 0x3F) {
        return false;
      } else if (((buffer[0] >> 7) == 1) & ((buffer[1] >> 7) == 0)) {
        // marker bits are ok, extract the values
        sensorNumber = (buffer[0] >> 4) & 0x7;
        level = ((buffer[0] & 0xF) << 6) | (buffer[1] & 0x3F);
        return true;
      } else {
        // marker bits are wrong. Assume a byte was dropped: drop first byte and try again
        buffer[0] = buffer[1];
        bytesRead = 1;
      }
    }
  }

  void PowerSensor::IOThread() {
    threadStarted.up();
    unsigned int sensorNumber;
    uint16_t level;
    while (readLevelFromDevice(sensorNumber, level)) {
      std::unique_lock<std::mutex> lock(mutex);
      sensors[sensorNumber].updateLevel(level);

      if (dumpFile != nullptr) {
        dumpCurrentPowerToFile();
      }
    }
  }

  void PowerSensor::startIOThread() {
    if (thread == nullptr) {
      thread = new std::thread(&PowerSensor::IOThread, this);
    }

    if (write(fd, "S", 1) != 1) {
      perror("write device");
      exit(1);
    }

    threadStarted.down();  // wait for the IOthread to run smoothly
  }

  void PowerSensor::stopIOThread() {
    if (thread != nullptr) {
      if (write(fd, "X", 1) != 1) {
        perror("write device");
        exit(1);
      }
      thread->join();
      delete thread;
      thread = nullptr;
    }
  }

  void PowerSensor::dump(const char* dumpFileName) {
    dumpFile = std::unique_ptr<std::ofstream>(dumpFileName != nullptr ? new std::ofstream(dumpFileName) : nullptr);
  }

  void PowerSensor::dumpCurrentPowerToFile() {
    // TODO: power calculation of each sensor pair
    std::unique_lock<std::mutex> lock(dumpFileMutex);
    // double totalPower = 0;
    double time = omp_get_wtime();
    static double previousTime = startTime;

    *dumpFile << "S " << time - startTime;
    *dumpFile << ' ' << 1e6 * (time - previousTime);
    previousTime = time;

    for (const Sensor &sensor: sensors) {
      if (sensor.inUse) {
        *dumpFile << ' ' << sensor.getValue();
      }
    }
    *dumpFile << std::endl;
    // *dumpFile << ' ' << totalPower << std::endl;

  }

  double PowerSensor::getPower(unsigned int pairID) const {
    double power = 1;
    for (uint8_t sensorID = 0; sensorID < MAX_SENSORS; sensorID++) {
      if ((getPairId(sensorID) == pairID) && sensors[sensorID].inUse()) {
          power *= sensors[sensorID].getValue();
        }
      }
    }
    return power;
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
