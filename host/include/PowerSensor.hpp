#ifndef __POWER_SENSOR_H
#define __POWER_SENSOR_H

#include "Semaphore.hpp"

#include <thread>
#include <fstream>

#include <inttypes.h>

namespace PowerSensor {

  const static unsigned MAX_SENSORS = 8;
  const static float VOLTAGE = 3.3;
  const static unsigned MAX_LEVEL = 1023;

  struct State {
    double consumedEnergy[MAX_SENSORS/2];
    double timeAtRead;
  };

  double Joules(const State &firstState, const State &secondState, int sensorID = -1 /* default: all sensors */);
  double seconds(const State &firstState, const State &secondState);
  double Watt(const State &firstState, const State &secondState, int sensorID = -1 /* default: all sensors */);

  class PowerSensor {
    public:
      PowerSensor(const char* device);
      ~PowerSensor();

      void dump(const char *dumpFileName); // dumpFileName == 0 --> stop dumping

      void setType(unsigned int sensorID, const char* type);
      void setVref(unsigned int sensorID, const float vref);
      void setSlope(unsigned int sensorID, const float slope);
      void setPairId(unsigned int sensorID, const uint8_t pairId);
      void setInUse(unsigned int sensorID, const bool inUse);

      void getType(unsigned int sensorID, char* type) const;
      float getVref(unsigned int sensorID) const;
      float getSlope(unsigned int sensorID) const;
      uint8_t getPairId(unsigned int sensorID) const;
      bool getInUse(unsigned int sensorID) const;

      double getWatt(unsigned int pairID) const;

    private:
      int fd;
      int openDevice(const char* device);

      void getActivePairs();
      bool pairsInUse[MAX_SENSORS/2];

      void readSensorsFromEEPROM();
      void writeSensorsToEEPROM();
      bool readLevelFromDevice(unsigned int &sensorNumber, uint16_t &level);

      std::unique_ptr<std::ofstream> dumpFile;
      void dumpCurrentWattToFile();

      Semaphore threadStarted;
      std::thread* thread;
      mutable std::mutex mutex, dumpFileMutex;
      double startTime;
      void IOThread();
      void startIOThread();
      void stopIOThread();

      struct Sensor {
        struct EEPROM {
          char type[16];
          float vref;
          float slope;
          uint8_t pairId;
          bool inUse;
        } __attribute__((packed));

        char type[16];
        float vref;
        float slope;
        uint8_t pairId;
        bool inUse;
        uint16_t level;
        double valueAtLastMeasurement;
        double timeAtLastMeasurement;
        void setType(const char* type);
        void setVref(const float vref);
        void setSlope(const float slope);
        void setPairId(const uint8_t PairId);
        void setInUse(const bool inUse);
        double getValue() const;
        void readFromEEPROM(int fd);
        void writeToEEPROM(int fd) const;
        void updateLevel(uint16_t level);
        void reset();
      } sensors[MAX_SENSORS];
  };


}  // namespace PowerSensor

#endif  // __POWER_SENSOR_H
