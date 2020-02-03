//  Copyright (C) 2016
//  ASTRON (Netherlands Institute for Radio Astronomy) / John W. Romein
//  P.O. Box 2, 7990 AA  Dwingeloo, the Netherlands

//  This file is part of PowerSensor.

//  PowerSensor is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.

//  PowerSensor is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with PowerSensor.  If not, see <http://www.gnu.org/licenses/>.


#if !defined POWER_SENSOR_H
#define POWER_SENSOR_H

#include "Semaphore.h"

#include <inttypes.h>

#include <fstream>
#include <mutex>
#include <thread>


namespace PowerSensor {


const static unsigned MAX_SENSORS = 5;


struct State
{
  double consumedEnergy[MAX_SENSORS];
  double timeAtRead;
};


class PowerSensor
{
  public:
    PowerSensor(const char *device = "/dev/ttyACM0");
    ~PowerSensor();

    State read() const;

    void dump(const char *dumpFileName); // dumpFileName == 0 --> stop dumping
    void mark(const State &, const char *name = 0, unsigned tag = 0) const;
    void mark(const State &start, const State &stop, const char *name = 0, unsigned tag = 0) const;

    float getVolt(unsigned sensorID) const;
    void  setVolt(unsigned sensorID, float volt);
    float getType(unsigned sensorID) const;
    void  setType(unsigned sensorID, float type);
    float getNullLevel(unsigned sensorID) const;
    void  setNullLevel(unsigned sensorID, float nullLevel);
    bool  inUse(unsigned sensorID) const;

  private:
    struct Sensor
    {
      struct EEPROM
      {
	float volt __attribute__((packed));
	float type __attribute__((packed));
	float nullLevel __attribute__((packed));
      };

      float    volt, type, nullLevel;
      double   weight;
      double   consumedEnergy;
      double   wattAtlastMeasurement;
      double   timeAtLastMeasurement;

      bool     inUse() const;
      void     readFromEEPROM(int fd), writeToEEPROM(int fd) const;
      void     setVolt(float), setType(float), setNullLevel(float);
      void     updateDerivedValues();
      void     updateLevel(int16_t level);
      double   totalEnergy(double now) const;
      double   currentWatt() const;
    } sensors[MAX_SENSORS];

    double        startTime;
    int		  fd;
    std::unique_ptr<std::ofstream> dumpFile;
    //volatile bool stop;
    mutable std::mutex mutex, dumpFileMutex;
    Semaphore     threadStarted;
    std::thread	  *thread;
    void	  startIOthread(), stopIOthread();

    int		  openDevice(const char *device);
    double	  totalEnergy() const;

    void	  readSensorsFromEEPROM(), writeSensorsToEEPROM();
    bool	  readLevelFromDevice(unsigned &sensorNumber, unsigned &level);
    void	  dumpCurrentWattToFile();
    void	  IOthread();

    void	  startCleanupProcess();
};


double Joules(const State &firstState, const State &secondState, int sensorID = -1 /* default: all sensors */);
double seconds(const State &firstState, const State &secondState);
double Watt(const State &firstState, const State &secondState, int sensorID = -1 /* default: all sensors */);

}

#endif
