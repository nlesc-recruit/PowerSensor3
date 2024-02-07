#pragma once

#include <inttypes.h>

#include <array>
#include <thread>
#include <fstream>
#include <queue>
#include <memory>
#include <string>

#include "Semaphore.hpp"

namespace PowerSensor3 {

static const unsigned MAX_SENSORS = 8;
static const unsigned MAX_PAIRS = MAX_SENSORS / 2;
static const float VOLTAGE = 3.3;
static const unsigned MAX_LEVEL = 1023;
static const std::string POWERSENSOR_VERSION = "1.3.4";

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
    void setPairName(unsigned int pairID, const std::string pairName);
    void setVref(unsigned int sensorID, const float vref);
    void setSensitivity(unsigned int sensorID, const float slope);
    void setInUse(unsigned int sensorID, const bool inUse);
    void setPolarity(unsigned int sensorID, const int polarity);

    std::string getType(unsigned int sensorID) const;
    std::string getPairName(unsigned int pairID) const;
    float getVref(unsigned int sensorID) const;
    float getSensitivity(unsigned int sensorID) const;
    bool getInUse(unsigned int sensorID) const;
    int getPolarity(unsigned int sensorID) const;

 private:
    static const unsigned MAX_TYPE_LENGTH = 16;
    static const unsigned MAX_PAIRNAME_LENGTH = 16;

    int fd;
    int pipe_fd;
    int openDevice(std::string device);
    std::queue<char> markers;
    void writeMarker();

    void initializeSensorPairs();
    void updateSensorPairs();

    inline char readCharFromDevice();
    inline void writeCharToDevice(char buffer);
    void readSensorsFromEEPROM();
    bool readLevelFromDevice(unsigned int* sensorNumber, uint16_t* level, unsigned int* marker);

    std::unique_ptr<std::ofstream> dumpFile;
    void dumpCurrentWattToFile();

    Semaphore threadStarted;
    std::thread* thread;
    mutable std::mutex mutex, dumpFileMutex;
    double startTime;
    unsigned int timestamp;
    void IOThread();
    void startIOThread();
    void stopIOThread();
    int startCleanupProcess();

    double totalEnergy(unsigned int pairID) const;

    struct Sensor {
      /** @brief Configuration of a single sensor as read from device EEPROM */
      struct EEPROM {
        /** @brief Sensor type */
        char type[MAX_TYPE_LENGTH];
        /** @brief Sensor pair name */
        char pairName[MAX_PAIRNAME_LENGTH];
        /** @brief Sensor reference voltage */
        float vref;
        /** @brief Sensor sensitivity (V/A for current sensors, unitless gain for voltage sensors) */
        float sensitivity;
        /** @brief Whether or not the sensor is in use */
        bool inUse;
      } __attribute__((packed));

      std::string type;
      std::string pairName;
      float vref;
      float sensitivity;
      bool inUse;
      uint16_t level;
      double valueAtLastMeasurement;
      void setType(const std::string type);
      void setPairName(const std::string pairName);
      void setVref(const float vref);
      void setSensitivity(const float slope);
      void setInUse(const bool inUse);
      void setPolarity(const int polarity);
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

}  // namespace PowerSensor3
