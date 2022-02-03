#ifndef __POWER_SENSOR_H
#define __POWER_SENSOR_H

#include <cstdint>

namespace PowerSensor {

  const static unsigned MAX_SENSORS = 8;

  class PowerSensor {
    public:
      PowerSensor(const char* device);
      ~PowerSensor();

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

    private:
      int fd;
      int openDevice(const char* device);
      void readSensorsFromEEPROM();
      void writeSensorsToEEPROM();

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
        void setType(const char* type);
        void setVref(const float vref);
        void setSlope(const float slope);
        void setPairId(const uint8_t PairId);
        void setInUse(const bool inUse);
        void readFromEEPROM(int fd);
        void writeToEEPROM(int fd) const;
      } sensors[MAX_SENSORS];
  };


}  // namespace PowerSensor

#endif  // __POWER_SENSOR_H
