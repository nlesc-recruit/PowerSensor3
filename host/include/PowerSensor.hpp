#pragma once

#include <inttypes.h>

#include <thread>
#include <fstream>
#include <queue>
#include <memory>
#include <string>

#include "Semaphore.hpp"

namespace PowerSensor {

static const unsigned MAX_SENSORS = 8;
static const unsigned MAX_PAIRS = MAX_SENSORS / 2;
static const float VOLTAGE = 3.3;
static const unsigned MAX_LEVEL = 1023;

/**
 * @brief Struct containing values of all active sensors at a single point in time
 *
 */
struct State {
  /** @brief Total energy consumption, per sensor */
  std::array<double, MAX_PAIRS> consumedEnergy;
  /** @brief Current, per sensor */
  std::array<double, MAX_PAIRS> current;
  /** @brief Voltage, per sensor */
  std::array<double, MAX_PAIRS> voltage;
  /** @brief Timestamp */
  double timeAtRead;
};

double Joules(const State &firstState, const State &secondState, int pairID = -1 /* default: all sensor pairs */);
double seconds(const State &firstState, const State &secondState);
double Watt(const State &firstState, const State &secondState, int pairID = -1 /* default: all sensor pairs */);
double Volt(const State &firstState, const State &secondState, int pairID);
double Ampere(const State &firstState, const State &secondState, int pairID);

/**
 * @brief The main PowerSensor class.
 *
 * Connects to a PowerSensor device and reads out its sensors continously using a light-overhead thread.
 * Sensor values can be obtained with the read method, or written to a file with the dump method
 *
 */
class PowerSensor {
 public:
    explicit PowerSensor(std::string device);
    ~PowerSensor();

    State read() const;

    void dump(const std::string dumpFileName);  // dumpFileName == 0 --> stop dumping
    void mark(char name);
    void mark(const State &startState, const State &stopState, const std::string name = 0, unsigned int tag = 0) const;

    void writeSensorsToEEPROM();
    void setType(unsigned int sensorID, const std::string type);
    void setVref(unsigned int sensorID, const float vref);
    void setSensitivity(unsigned int sensorID, const float slope);
    void setInUse(unsigned int sensorID, const bool inUse);

    std::string getType(unsigned int sensorID) const;
    float getVref(unsigned int sensorID) const;
    float getSensitivity(unsigned int sensorID) const;
    bool getInUse(unsigned int sensorID) const;

 private:
    static const unsigned MAX_TYPE_LENGTH = 16;

    int fd;
    int openDevice(std::string device);
    std::queue<char> markers;
    void writeMarker();

    unsigned int numActiveSensors;
    void initializeSensorPairs();
    void updateSensorPairs();

    void readSensorsFromEEPROM();
    bool readLevelFromDevice(unsigned int* sensorNumber, uint16_t* level, unsigned int* marker);

    std::unique_ptr<std::ofstream> dumpFile;
    void dumpCurrentWattToFile();

    Semaphore threadStarted;
    std::thread* thread;
    mutable std::mutex mutex, dumpFileMutex;
    double startTime;
    void IOThread();
    void startIOThread();
    void stopIOThread();
    void startCleanupProcess();

    double totalEnergy(unsigned int pairID) const;

    struct Sensor {
      /** @brief Configuration of a single sensor as read from device EEPROM */
      struct EEPROM {
        /** @brief Sensor type */
        char type[MAX_TYPE_LENGTH];
        /** @brief Sensor reference voltage */
        float vref;
        /** @brief Sensor sensitivity (V/A for current sensors, unitless gain for voltage sensors) */
        float sensitivity;
        /** @brief Whether or not the sensor is in use */
        bool inUse;
      } __attribute__((packed));

      std::string type;
      float vref;
      float sensitivity;
      bool inUse;
      uint16_t level;
      double valueAtLastMeasurement;
      void setType(const std::string type);
      void setVref(const float vref);
      void setSensitivity(const float slope);
      void setPairId(const uint8_t PairId);
      void setInUse(const bool inUse);
      double getValue() const;
      void readFromEEPROM(int fd);
      void writeToEEPROM(int fd) const;
      void updateLevel(uint16_t level);
      void reset();
    } sensors[MAX_SENSORS];

    struct SensorPair {
      double currentAtLastMeasurement;
      double voltageAtLastMeasurement;
      double wattAtLastMeasurement;
      double timeAtLastMeasurement;
      double consumedEnergy;
      bool inUse;
    } sensorPairs[MAX_PAIRS];
};

}  // namespace PowerSensor
