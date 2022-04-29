#pragma once

#include <inttypes.h>

#include <thread>
#include <fstream>
#include <queue>
#include <memory>

#include "Semaphore.hpp"

namespace PowerSensor {

const unsigned MAX_SENSORS = 8;
const unsigned MAX_PAIRS = MAX_SENSORS / 2;
const float VOLTAGE = 3.3;
const unsigned MAX_LEVEL = 1023;

struct State {
  double consumedEnergy[MAX_PAIRS];
  double current[MAX_PAIRS];
  double voltage[MAX_PAIRS];
  double timeAtRead;
};

double Joules(const State &firstState, const State &secondState, int pairID = -1 /* default: all sensor pairs */);
double seconds(const State &firstState, const State &secondState);
double Watt(const State &firstState, const State &secondState, int pairID = -1 /* default: all sensor pairs */);
double Volt(const State &firstState, const State &secondState, int pairID);
double Ampere(const State &firstState, const State &secondState, int pairID);

class PowerSensor {
 public:
    explicit PowerSensor(const char* device);
    ~PowerSensor();

    State read() const;

    void dump(const char *dumpFileName);  // dumpFileName == 0 --> stop dumping
    void mark(char name);
    void mark(const State &startState, const State &stopState, const char *name = 0, unsigned int tag = 0) const;

    void setType(unsigned int sensorID, const char* type);
    void setVref(unsigned int sensorID, const float vref);
    void setSensitivity(unsigned int sensorID, const float slope);
    void setInUse(unsigned int sensorID, const bool inUse);

    void getType(unsigned int sensorID, char* type) const;
    float getVref(unsigned int sensorID) const;
    float getSensitivity(unsigned int sensorID) const;
    bool getInUse(unsigned int sensorID) const;

 private:
    int fd;
    int openDevice(const char* device);
    std::queue<char> markers;
    void writeMarker();

    unsigned int numActiveSensors;
    void initializeSensorPairs();
    void updateSensorPairs();

    void readSensorsFromEEPROM();
    void writeSensorsToEEPROM();
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
        char type[16];
        float vref;
        float sensitivity;
        bool inUse;
      } __attribute__((packed));

      char type[16];
      float vref;
      float sensitivity;
      bool inUse;
      uint16_t level;
      double valueAtLastMeasurement;
      void setType(const char* type);
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
