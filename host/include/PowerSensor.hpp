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

struct State {
  std::array<double, MAX_PAIRS> consumedEnergy;
  std::array<double, MAX_PAIRS> current;
  std::array<double, MAX_PAIRS> voltage;
  double timeAtRead;
};

double Joules(const State &firstState, const State &secondState, int pairID = -1 /* default: all sensor pairs */);
double seconds(const State &firstState, const State &secondState);
double Watt(const State &firstState, const State &secondState, int pairID = -1 /* default: all sensor pairs */);
double Volt(const State &firstState, const State &secondState, int pairID);
double Ampere(const State &firstState, const State &secondState, int pairID);

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
      struct EEPROM {
        char type[MAX_TYPE_LENGTH];
        float vref;
        float sensitivity;
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
