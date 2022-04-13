#include <fcntl.h>
#include <omp.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <cstring>

#include "PowerSensor.hpp"


namespace PowerSensor {

  void checkPairID(int pairID) {
    if (pairID >= (signed)MAX_PAIRS) {
      std::cerr << "Invalid pairID: " << pairID << ", maximum value is " << MAX_PAIRS - 1 << std::endl;
      exit(1);
    }
  }

  double Joules(const State &firstState, const State &secondState, int pairID) {
    checkPairID(pairID);

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
    return Joules(firstState, secondState, pairID) / seconds(firstState, secondState);
  }

  double Volt(const State &firstState, const State &secondState, int pairID) {
    checkPairID(pairID);
    return .5 * (firstState.voltage[pairID] + secondState.voltage[pairID]);
  }

  double Ampere(const State &firstState, const State &secondState, int pairID) {
    checkPairID(pairID);
    return .5 * (firstState.current[pairID] + secondState.current[pairID]);
  }

  PowerSensor::PowerSensor(const char* device):
    fd(openDevice(device)),
    thread(nullptr),
    startTime(omp_get_wtime()) {
      startCleanupProcess();
      readSensorsFromEEPROM();
      initializeSensorPairs();
      startIOThread();
    }

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

    for (uint8_t pairID=0; pairID < MAX_PAIRS; pairID++) {
      state.consumedEnergy[pairID] = sensorPairs[pairID].consumedEnergy;
      state.current[pairID] = sensorPairs[pairID].currentAtLastMeasurement;
      state.voltage[pairID] = sensorPairs[pairID].voltageAtLastMeasurement;
    }
    return state;
  }

  int PowerSensor::openDevice(const char* device) {
    int fileDescriptor;

    // opens the file specified by pathname;
    if ((fileDescriptor = open(device, O_RDWR)) < 0) {
      perror(device);
      exit(1);
    }
    // block if an incompatible lock is held by another process;
    if (flock(fileDescriptor, LOCK_EX) < 0) {
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
    for (Sensor& sensor : sensors) {
      sensor.readFromEEPROM(fd);
    }
  }

  void PowerSensor::writeSensorsToEEPROM() {
    if (write(fd, "W", 1) != 1) {
      perror("write device");
      exit(1);
    }
    for (const Sensor& sensor : sensors) {
      sensor.writeToEEPROM(fd);
      usleep(10000);
    }
  }

  void PowerSensor::initializeSensorPairs() {
    numActiveSensors = 0;
    for (uint8_t pairID = 0; pairID < MAX_PAIRS; pairID++) {
      sensorPairs[pairID].timeAtLastMeasurement = startTime;
      sensorPairs[pairID].wattAtLastMeasurement = 0;
      sensorPairs[pairID].consumedEnergy = 0;
      sensorPairs[pairID].currentAtLastMeasurement = 0;
      sensorPairs[pairID].voltageAtLastMeasurement = 0;


      bool currentSensorActive = sensors[2*pairID].inUse;
      bool voltageSensorActive = sensors[2*pairID+1].inUse;

      if (currentSensorActive && voltageSensorActive) {
        sensorPairs[pairID].inUse = true;
        numActiveSensors += 2;
      } else if (currentSensorActive ^ voltageSensorActive) {
        std::cerr << "Found incompatible sensor pair: current sensor (ID " << 2*pairID << ") "
        "is " << (currentSensorActive ? "" : "not ") << "active, while "
        "voltage sensor (ID " << 2*pairID+1 << ") is " << (voltageSensorActive ? "" : "not ") << "active. "
        "Please check sensor configuration." << std::endl;
      } else {
        sensorPairs[pairID].inUse = false;
      }
    }
  }

  bool PowerSensor::readLevelFromDevice(unsigned int* sensorNumber, uint16_t* level, unsigned int* marker) {
      // buffer for one set of sensor data (2 bytes)
      uint8_t buffer[2];
      ssize_t retVal, bytesRead = 0;
      // loop exits when a valid value has been read from the device
      while (true) {
        // read full buffer
        do {
          if ((retVal = ::read(fd, reinterpret_cast<char*>(&buffer) + bytesRead, sizeof(buffer) - bytesRead)) < 0) {
            perror("read");
            exit(1);
          }
        } while ((bytesRead += retVal) < sizeof buffer);

        // buffer is full, check if stop was received
        if (buffer[0] == 0xFF && buffer[1] == 0x3F) {
          return false;
        } else if (((buffer[0] >> 7) == 1) & ((buffer[1] >> 7) == 0)) {
          // marker bits are ok, extract the values
          *sensorNumber = (buffer[0] >> 4) & 0x7;
          *level = ((buffer[0] & 0xF) << 6) | (buffer[1] & 0x3F);
          *marker |= (buffer[1] >> 6) & 0x1;
          return true;
        } else {
          // marker bits are wrong. Assume a byte was dropped: drop first byte and try again
          buffer[0] = buffer[1];
          bytesRead = 1;
        }
      }
    }

  void PowerSensor::mark(char name) {
    markers.push(name);
    if (write(fd, "M", 1) < 0) {
      perror("write device");
      exit(1);
    }
  }

  void PowerSensor::writeMarker() {
    std::unique_lock<std::mutex> lock(dumpFileMutex);
    *dumpFile << "M " << markers.front() << std::endl;
    markers.pop();
  }

  void PowerSensor::mark(const State &startState, const State &stopState, const char* name, unsigned int tag) const {
    if (dumpFile != nullptr) {
      std::unique_lock<std::mutex> lock(dumpFileMutex);
      *dumpFile << "M " << startState.timeAtRead - startTime << ' ' << stopState.timeAtRead - startTime << ' ' \
        << tag << " \"" << (name == nullptr ? "" : name) << '"' << std::endl;
    }
  }

  void PowerSensor::IOThread() {
    threadStarted.up();
    unsigned int sensorNumber, marker = 0, sensorsRead = 0;
    uint16_t level;

    while (readLevelFromDevice(&sensorNumber, &level, &marker)) {
      std::unique_lock<std::mutex> lock(mutex);
      sensors[sensorNumber].updateLevel(level);
      sensorsRead++;

      if (sensorsRead >= numActiveSensors) {
        sensorsRead = 0;
        updateSensorPairs();

        if (dumpFile != nullptr) {
          if (marker != 0) {
            writeMarker();
            marker = 0;
          }
          dumpCurrentWattToFile();
        }
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
    *dumpFile << ' ' << static_cast<int>(1e6 * (time - previousTime));
    previousTime = time;

    for (uint8_t pairID=0; pairID < MAX_PAIRS; pairID++) {
      if (sensorPairs[pairID].inUse) {
        totalWatt += sensorPairs[pairID].wattAtLastMeasurement;
        *dumpFile << ' ' << sensorPairs[pairID].wattAtLastMeasurement;
      }
    }
    *dumpFile << ' ' << totalWatt << std::endl;
  }

  void PowerSensor::updateSensorPairs() {
    for (unsigned int pairID=0; pairID < MAX_PAIRS; pairID++) {
      if (sensorPairs[pairID].inUse) {
        Sensor currentSensor = sensors[2*pairID];
        Sensor voltageSensor = sensors[2*pairID+1];
        SensorPair& sensorPair = sensorPairs[pairID];
        double now = omp_get_wtime();

        sensorPair.currentAtLastMeasurement = currentSensor.valueAtLastMeasurement;
        sensorPair.voltageAtLastMeasurement = voltageSensor.valueAtLastMeasurement;
        sensorPair.wattAtLastMeasurement = currentSensor.valueAtLastMeasurement * voltageSensor.valueAtLastMeasurement;
        sensorPair.consumedEnergy += sensorPair.wattAtLastMeasurement * (now - sensorPair.timeAtLastMeasurement);
        sensorPair.timeAtLastMeasurement = now;
      }
    }
  }

  void PowerSensor::startCleanupProcess() {
    /*
    spawn child process to make sure that the device receives a 'T'
    to stop sending data, no matter how the application terminates
    */

    int pipe_fds[2];

    if (pipe(pipe_fds) < 0) {
      perror("pipe");
      exit(1);
    }

    switch (fork()) {
      case -1:
        perror("fork");
        exit(1);

      case 0:
        // detach from the parent process, so signals to the parent are not caught by the child
        setsid();

        // close all file descriptors, except pipe read end and device fd
        for (int i = 3, last = getdtablesize(); i < last; i++) {
          if (i != fd && i != pipe_fds[0]) {
            close(i);
          }
        }

      // wait until parent closes pipe_fds[1] so that read fails
      char byte;
      ::read(pipe_fds[0], &byte, sizeof byte);

      // tell device to stop sending data
      write(fd, "T", 1);

      // drain garbage
      usleep(100000);
      tcflush(fd, TCIFLUSH);

      exit(0);

    default:
      close(pipe_fds[0]);
    }
  }

  double PowerSensor::totalEnergy(unsigned int pairID) const {
    double energy = sensorPairs[pairID].wattAtLastMeasurement *
      (omp_get_wtime() - sensorPairs[pairID].timeAtLastMeasurement);

    return sensorPairs[pairID].consumedEnergy + energy;
  }

  void PowerSensor::getType(unsigned int sensorID, char* type) const {
    strlcpy(type, sensors[sensorID].type, sizeof sensors[sensorID].type);
  }

  float PowerSensor::getVref(unsigned int sensorID) const {
    return sensors[sensorID].vref;
  }

  float PowerSensor::getSlope(unsigned int sensorID) const {
    return sensors[sensorID].slope;
  }

  bool PowerSensor::getInUse(unsigned int sensorID) const {
    return sensors[sensorID].inUse;
  }

  void PowerSensor::setType(unsigned int sensorID, const char* type) {
    sensors[sensorID].setType(type);
    writeSensorsToEEPROM();
  }

  void PowerSensor::setVref(unsigned int sensorID, const float vref) {
    sensors[sensorID].setVref(vref);
    writeSensorsToEEPROM();
  }

  void PowerSensor::setSlope(unsigned int sensorID, const float slope) {
    sensors[sensorID].setSlope(slope);
    writeSensorsToEEPROM();
  }

  void PowerSensor::setInUse(unsigned int sensorID, const bool inUse) {
    sensors[sensorID].setInUse(inUse);
    writeSensorsToEEPROM();
  }

}  // namespace PowerSensor
