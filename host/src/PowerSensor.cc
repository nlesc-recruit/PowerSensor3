#include <fcntl.h>
#include <omp.h>
#include <termios.h>
#include <unistd.h>
#include <sys/file.h>

#include <chrono>
#include <iostream>
#include <cstring>

#include "PowerSensor.hpp"


namespace PowerSensor3 {

  /**
   * @brief Check if the given id of a sensor pair is valid
   *
   * @param pairID
   */
  void checkPairID(int pairID) {
    if (pairID >= (signed)MAX_PAIRS) {
      std::cerr << "Invalid pairID: " << pairID << ", maximum value is " << MAX_PAIRS - 1 << std::endl;
      exit(1);
    }
  }

  /**
   * @brief Get energy usage (J) between two states
   *
   * @param firstState
   * @param secondState
   * @param pairID Sensor pair to get energy usage for, -1 to get the total energy usage
   * @return double
   */
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

  /**
   * @brief Get time difference (s) between two states
   *
   * @param firstState
   * @param secondState
   * @return double
   */
  double seconds(const State &firstState, const State &secondState) {
    return secondState.timeAtRead - firstState.timeAtRead;
  }

  /**
   * @brief Get average power (W) between two states
   *
   * @param firstState
   * @param secondState
   * @param pairID Sensor pair to get power for, -1 to get the total power
   * @return double
   */
  double Watt(const State &firstState, const State &secondState, int pairID) {
    return Joules(firstState, secondState, pairID) / seconds(firstState, secondState);
  }

  /**
   * @brief Construct a new Power Sensor:: Power Sensor object
   *
   * @param device path to device, e.g. /dev/ttyACM1
   */
  PowerSensor::PowerSensor(std::string device):
    fd(openDevice(device)),
    pipe_fd(startCleanupProcess()),
    thread(nullptr),
    startTime(omp_get_wtime()) {
      readSensorsFromEEPROM();
      initializeSensorPairs();
      startIOThread();
    }

  /**
   * @brief Destroy the Power Sensor:: Power Sensor object
   *
   */
  PowerSensor::~PowerSensor() {
    stopIOThread();
    if (close(pipe_fd)) {
      perror("close child pipe fd");
    }

    if (close(fd)) {
      perror("close device");
    }
  }

  /**
   * @brief Read sensor values
   *
   * @return State
   */
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

  /**
   * @brief Connect to PowerSensor device
   *
   * @param device path to device, e.g. /dev/ttyACM1
   * @return int file descriptor
   */
  int PowerSensor::openDevice(std::string device) {
    int fileDescriptor;

    // opens the file specified by pathname;
    if ((fileDescriptor = open(device.c_str(), O_RDWR)) < 0) {
      perror(device.c_str());
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

  /**
   * @brief Obtain sensor configuration from device EEPROM
   *
   */
  void PowerSensor::readSensorsFromEEPROM() {
    if (write(fd, "R", 1) != 1) {
      perror("write device");
      exit(1);
    }
    for (Sensor& sensor : sensors) {
      sensor.readFromEEPROM(fd);
    }
  }

  /**
   * @brief Write sensor configuration to device EEPROM
   *
   */
  void PowerSensor::writeSensorsToEEPROM() {
    // ensure no data is streaming to host
    stopIOThread();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // drain any remaining incoming data
    tcflush(fd, TCIFLUSH);
    // signal device to receive EEPROM data
    if (write(fd, "W", 1) != 1) {
      perror("write device");
      exit(1);
    }
    // send EEPROM data
    for (const Sensor& sensor : sensors) {
      sensor.writeToEEPROM(fd);
    }
    // wait for device to finish processing the new EEPROM data
    char buffer;
    ssize_t bytesRead;
    do {
      if ((bytesRead = ::read(fd, &buffer, 1)) < 0) {
        perror("read");
        exit(1);
      }
    } while ((bytesRead) < 1);

    if (buffer != 'D') {
      std::cerr << "Expected to receive 'D' from device after writing configuration, but got " << buffer << std::endl;
      exit(1);
    }
    // restart IO thread
    startIOThread();
  }

  /**
   * @brief Set up sensor pairs based on sensor configuration
   *
   */
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

  /**
   * @brief Read single sensor value from device
   *
   * @param sensorNumber ID of sensor that was read
   * @param level raw level of sensor
   * @param marker whether or not a marker should be written to the output file
   * @return bool whether or not the device sent a stop signal
   */
  bool PowerSensor::readLevelFromDevice(unsigned int* sensorNumber, uint16_t* level, unsigned int* marker) {
      // buffer for one set of sensor data (2 bytes)
      uint8_t buffer[2];
      unsigned int retVal, bytesRead = 0;
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

  /**
   * @brief Instruct device to mark the next sensor value
   *
   * @param name marker character
   */
  void PowerSensor::mark(char name) {
    markers.push(name);
    if (write(fd, "M", 1) < 0) {
      perror("write device");
      exit(1);
    }
  }

  /**
   * @brief Write marker character to output file
   *
   */
  void PowerSensor::writeMarker() {
    std::unique_lock<std::mutex> lock(dumpFileMutex);
    *dumpFile << "M " << markers.front() << std::endl;
    markers.pop();
  }

  /**
   * @brief Write custom marker to dump file
   *
   * @param startState State used to get start time
   * @param stopState State used to get end time
   * @param name name of the marker (string)
   * @param tag id of the marker (int)
   */
  void PowerSensor::mark(const State &startState, const State &stopState, std::string name, unsigned int tag) const {
    if (dumpFile != nullptr) {
      std::unique_lock<std::mutex> lock(dumpFileMutex);
      *dumpFile << "M " << startState.timeAtRead - startTime << ' ' << stopState.timeAtRead - startTime << ' ' \
        << tag << " \"" << name << '"' << std::endl;
    }
  }

  /**
   * @brief thread to continuously read sensor values from device
   *
   */
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

  /**
   * @brief Start a new IO thread on the host and instruct device to start sending sensor values
   *
   */
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

  /**
   * @brief Instruct device to stop sending sensor values.
   *
   * The device stops sending sensor values and then sends a stop signal
   * The host IO thread interprets this and stops.
   *
   */
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

  /**
   * @brief Enable dumping of sensor values to the given output file.
   *
   * @param dumpFileName
   */
  void PowerSensor::dump(std::string dumpFileName) {
    std::unique_lock<std::mutex> lock(dumpFileMutex);
    dumpFile = std::unique_ptr<std::ofstream>(dumpFileName.empty() ? nullptr: new std::ofstream(dumpFileName));
    if (!dumpFileName.empty()) {
      *dumpFile << "marker time dt_micro";
      for (unsigned int pairID=0; pairID < MAX_PAIRS; pairID++) {
        if (sensorPairs[pairID].inUse)
          *dumpFile << " current" << pairID << " voltage" << pairID << " power" << pairID;
      }
      *dumpFile << " power_total" << std::endl;
    }
  }

  /**
   * @brief Write sensor values to the output file
   *
   */
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
        *dumpFile << ' ' << sensorPairs[pairID].currentAtLastMeasurement;
        *dumpFile << ' ' << sensorPairs[pairID].voltageAtLastMeasurement;
        *dumpFile << ' ' << sensorPairs[pairID].wattAtLastMeasurement;
      }
    }
    *dumpFile << ' ' << totalWatt << std::endl;
  }

  /**
   * @brief Update sensor pairs based on values of individual sensors
   *
   */
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

  /**
   * @brief Spawn child process to make sure that the device stops sending
   * sensor values, no matter how the application terminates.
   *
   */
  int PowerSensor::startCleanupProcess() {
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        tcflush(fd, TCIFLUSH);

        exit(0);

    default:
      return pipe_fds[1];
    }
  }

  /**
   * @brief Calculate total energy usage of given sensor Pair
   *
   * @param pairID
   * @return double Energy usage in Joules
   */
  double PowerSensor::totalEnergy(unsigned int pairID) const {
    double energy = sensorPairs[pairID].wattAtLastMeasurement *
      (omp_get_wtime() - sensorPairs[pairID].timeAtLastMeasurement);

    return sensorPairs[pairID].consumedEnergy + energy;
  }

  /**
   * @brief Get type of given sensor
   *
   * @param sensorID
   * @return std::string
   */
  std::string PowerSensor::getType(unsigned int sensorID) const {
    return sensors[sensorID].type;
  }

  /**
   * @brief Get reference voltage (V) of given sensor
   *
   * @param sensorID
   * @return float
   */
  float PowerSensor::getVref(unsigned int sensorID) const {
    return sensors[sensorID].vref;
  }

  /**
   * @brief Get sensitivity of given sensor, in V/A (current sensors) of unitless gain (voltage sensors)
   *
   * @param sensorID
   * @return float
   */
  float PowerSensor::getSensitivity(unsigned int sensorID) const {
    return sensors[sensorID].sensitivity;
  }

  /**
   * @brief Get whether or not the given sensor is in use
   *
   * @param sensorID
   * @return bool
   */
  bool PowerSensor::getInUse(unsigned int sensorID) const {
    return sensors[sensorID].inUse;
  }

  /**
   * @brief Set type of given sensor
   *
   * @param sensorID
   * @param type
   */
  void PowerSensor::setType(unsigned int sensorID, const std::string type) {
    sensors[sensorID].setType(type);
  }

  /**
   * @brief Set reference voltage (V) of given sensor
   *
   * @param sensorID
   * @param vref
   */
  void PowerSensor::setVref(unsigned int sensorID, const float vref) {
    sensors[sensorID].setVref(vref);
  }

  /**
   * @brief Set sensitivity of given sensor, in V/A (current sensors) of unitless gain (voltage sensors)
   *
   * @param sensorID
   * @param sensitivity
   */
  void PowerSensor::setSensitivity(unsigned int sensorID, const float sensitivity) {
    sensors[sensorID].setSensitivity(sensitivity);
  }

  /**
   * @brief Set whether or not the given sensor is in use
   *
   * @param sensorID
   * @param inUse
   */
  void PowerSensor::setInUse(unsigned int sensorID, const bool inUse) {
    sensors[sensorID].setInUse(inUse);
  }

}  // namespace PowerSensor3
