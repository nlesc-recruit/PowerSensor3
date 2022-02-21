#include "PowerSensor.hpp"

#include <iostream>
#include <cstring>

#include <fcntl.h>
#include <omp.h>
#include <termios.h>
#include <unistd.h>


namespace PowerSensor {

  double Joules(const State &firstState, const State &secondState, int pairID) {
    if (pairID >= (signed)MAX_SENSORS/2) {
      std::cerr << "Invalid pairID: " << pairID << ", maximum value is " << MAX_SENSORS/2 << std::endl;
      exit(1);
    }

    if (pairID >= 0) {
      return secondState.consumedEnergy[pairID] - firstState.consumedEnergy[pairID];
    }

    double joules = 0;
    for (double consumedEnergy : secondState.consumedEnergy) {
      joules += consumedEnergy;
    }
    for (double consumedEnergy : firstState.consumedEnergy) {
      joules -= consumedEnergy;
    }
    return joules;
  }

  double seconds(const State &firstState, const State &secondState) {
    return secondState.timeAtRead - firstState.timeAtRead;
  }

  double Watt(const State &firstState, const State &secondState, int pairID) {
    return Joules(firstState, secondState) / seconds(firstState, secondState);
  }

  PowerSensor::PowerSensor(const char* device):
    fd(openDevice(device)),
    thread(nullptr),
    startTime(omp_get_wtime())
    {
      readSensorsFromEEPROM();
      getActivePairs();
      startIOThread();
    };

  PowerSensor::~PowerSensor() {
    stopIOThread();

    if (close(fd)) {
      perror("close device");
    }
  }

  State PowerSensor::read() const {
    State state;

    std::unique_lock<std::mutex> lock(mutex);
    state.timeAtRead = omp_get_wtime();

    for (uint8_t pairID=0; pairID < MAX_SENSORS/2; pairID++) {
      state.consumedEnergy[pairID] = totalEnergy(pairID);
    }
    return state;
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

  void PowerSensor::getActivePairs() {
    numActiveSensors = 0;
    for (uint8_t pairID = 0; pairID < MAX_SENSORS / 2; pairID++) {
      bool currentSensorActive = sensors[2*pairID].inUse;
      bool voltageSensorActive = sensors[2*pairID+1].inUse;
      if (currentSensorActive && voltageSensorActive) {
        sensorPairs[pairID].inUse = true;
        numActiveSensors += 2;
      } else if (currentSensorActive ^ voltageSensorActive) {
        std::cerr << "Found incompatible sensor pair: current sensor (ID " << 2*pairID << ") is " << (currentSensorActive ? "" : "not ") << "active, while ";
        std::cerr << "voltage sensor (ID " << 2*pairID+1 << ") is " << (voltageSensorActive ? "" : "not ") << "active. ";
        std::cerr << "Please check sensor configuration." << std::endl;
        exit(1);
      } else {
        sensorPairs[pairID].inUse = false;
      }
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
    unsigned int sensorsRead = 0;

    while (readLevelFromDevice(sensorNumber, level)) {
      std::unique_lock<std::mutex> lock(mutex);
      sensors[sensorNumber].updateLevel(level);
      sensorsRead++;

      if ((dumpFile != nullptr) & (sensorsRead >= numActiveSensors)) {
        sensorsRead = 0;
        dumpCurrentWattToFile();
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

  void PowerSensor::dumpCurrentWattToFile() {
    std::unique_lock<std::mutex> lock(dumpFileMutex);
    double totalWatt = 0;
    double time = omp_get_wtime();
    static double previousTime = startTime;

    *dumpFile << "S " << time - startTime;
    *dumpFile << ' ' << 1e6 * (time - previousTime);
    previousTime = time;

    for (uint8_t pairID=0; pairID < MAX_SENSORS/2; pairID++) {
      if (sensorPairs[pairID].inUse) {
        totalWatt += getWatt(pairID);
        *dumpFile << ' ' << getWatt(pairID);
      }
    }
    *dumpFile << ' ' << totalWatt << std::endl;
  }

  double PowerSensor::getWatt(unsigned int pairID) const {
    if (!sensorPairs[pairID].inUse) {
      return -1;
    }
    Sensor currentSensor = sensors[2*pairID];
    Sensor voltageSensor = sensors[2*pairID+1];
    return currentSensor.getValue() * voltageSensor.getValue();
  }

  double PowerSensor::totalEnergy(unsigned int pairID) const {
    if (!sensorPairs[pairID].inUse) {
      return -1;
    }
    double power = getWatt(pairID);

    double currentTimeAtLastMeasurement = sensors[2*pairID].timeAtLastMeasurement;
    double voltageTimeAtLastMeasurement = sensors[2*pairID+1].timeAtLastMeasurement;
    // need to keep track of energy usage per pair, this only returns
    // energy usage since last measurement
    double seconds = omp_get_wtime() - .5 * (currentTimeAtLastMeasurement + voltageTimeAtLastMeasurement);
    return power * seconds;
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
