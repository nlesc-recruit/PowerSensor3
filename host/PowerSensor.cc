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

#include "PowerSensor.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <byteswap.h>
#include <errno.h>
#include <fcntl.h>
#include <omp.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

namespace PowerSensor
{

  void PowerSensor::Sensor::readFromEEPROM(int fd)
  {
    struct EEPROM eeprom;
    ssize_t retval, bytesRead = 0;

    do
    {
      if ((retval = ::read(fd, (char *)&eeprom + bytesRead, sizeof eeprom - bytesRead)) < 0)
      {
        perror("read device");
        exit(1);
      }
    } while ((bytesRead += retval) < sizeof eeprom);

#if defined __BIG_ENDIAN__
    eeprom.volt = __bswap_32(eeprom.volt);
    eeprom.type = __bswap_32(eeprom.type);
    eeprom.nullLevel = __bswap_32(eeprom.nullLevel);
#endif
    std::cout << "V: " << eeprom.volt << std::endl;
    setVolt(eeprom.volt);
    std::cout << "T: " << eeprom.type << std::endl;
    setType(eeprom.type);
    std::cout << "N: " << eeprom.nullLevel << std::endl;
    setNullLevel(eeprom.nullLevel);
    std::cout << " " << std::endl;
  }

  void PowerSensor::Sensor::writeToEEPROM(int fd) const
  {
    struct EEPROM eeprom;

    eeprom.volt = volt;
    eeprom.type = type;
    eeprom.nullLevel = nullLevel;

#if defined __BIG_ENDIAN__
    eeprom.volt = __bswap_32(eeprom.volt);
    eeprom.type = __bswap_32(eeprom.type);
    eeprom.nullLevel = __bswap_32(eeprom.nullLevel);
#endif

    ssize_t retval, bytesWritten = 0;

    do
    {
      if ((retval = ::write(fd, (char *)&eeprom + bytesWritten, sizeof eeprom - bytesWritten)) < 0)
      {
        perror("write device");
        exit(1);
      }
    } while ((bytesWritten += retval) < sizeof eeprom);
  }

  void PowerSensor::Sensor::updateDerivedValues()
  {
    weight = volt != 0 ? 2.5 / 776 * volt / type : 0;
    consumedEnergy = 0;
    wattAtlastMeasurement = 0;
    timeAtLastMeasurement = omp_get_wtime();
  }

  void PowerSensor::Sensor::setVolt(float volt)
  {
    this->volt = volt;
    updateDerivedValues();
  }

  void PowerSensor::Sensor::setType(float type)
  {
    this->type = type;
    updateDerivedValues();
  }

  void PowerSensor::Sensor::setNullLevel(float nullLevel)
  {
    this->nullLevel = nullLevel;
    updateDerivedValues();
  }

  int PowerSensor::openDevice(const char *device)
  {
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

    // sets the input baud rate;
    cfsetispeed(&terminalOptions, B4000000);

    // sets the output baud rate;
    cfsetospeed(&terminalOptions, B4000000);

    // set control mode flags;
    terminalOptions.c_cflag |= CLOCAL | CREAD | CS8;
    terminalOptions.c_cflag |= (PARENB | PARODD);		// off

    // set input mode flags;
    terminalOptions.c_iflag = 0;
    terminalOptions.c_iflag |= IGNBRK;			// off
    //terminalOptions.c_iflag |=(IXON | IXOFF | IXANY);		// off

    // clear local mode flag
    terminalOptions.c_lflag = 0;

    // clear output mode flag;
    terminalOptions.c_oflag = 0;

    // set control characters;
    terminalOptions.c_cc[VMIN] = 2;
    terminalOptions.c_cc[VTIME] = 0;

    // commit the options;
    tcsetattr(fileDescriptor, TCSANOW, &terminalOptions);

    // flush anything already in the serial buffer;
    tcflush(fileDescriptor, TCIFLUSH);

    return fileDescriptor;
  }

  PowerSensor::PowerSensor(const char *device)
      : startTime(omp_get_wtime()),
        fd(openDevice(device)),
        thread(nullptr)
  {
    startCleanupProcess();
    readSensorsFromEEPROM();
    startIOthread();
  }

  PowerSensor::~PowerSensor()
  {
    stopIOthread();

    if (close(fd))
      perror("close device");
  }

  void PowerSensor::readSensorsFromEEPROM()
  {
    if (write(fd, "R", 1) != 1)
    {
      perror("write device");
      exit(1);
    }

    for (Sensor &sensor : sensors)
      sensor.readFromEEPROM(fd);
  }

  void PowerSensor::writeSensorsToEEPROM()
  {
    stopIOthread();

    if (write(fd, "W", 1) != 1)
    {
      perror("write device");
      exit(1);
    }

#if defined UNO
    struct termios options;

    usleep(200000);
    tcgetattr(fd, &options);
    cfsetospeed(&options, B115200);
    tcsetattr(fd, TCSANOW, &options);
#endif

    for (const Sensor &sensor : sensors)
      sensor.writeToEEPROM(fd);

#if defined UNO
    usleep(200000);
    tcgetattr(fd, &options);
    cfsetospeed(&options, B2000000);
    tcsetattr(fd, TCSANOW, &options);
#endif

    startIOthread();
  }

  void PowerSensor::startCleanupProcess()
  {
    // spawn child process to make sure that the Arduino receives a 'T', no matter how the application terminates

    int pipe_fds[2];

    if (pipe(pipe_fds) < 0)
    {
      perror("pipe");
      exit(1);
    }

    switch (fork())
    {
      ssize_t retval;

    case -1:
      perror("fork");
      exit(1);

    case 0: // detach from the parent process, so signals to the parent are not caught by the child
      setsid();

      // close all file descriptors, except pipe read end and Arduino fd
      for (int i = 3, last = getdtablesize(); i < last; i++)
        if (i != fd && i != pipe_fds[0])
          close(i);

      // wait until parent closes pipe_fds[1] so that read fails
      char byte;
      retval = ::read(pipe_fds[0], &byte, sizeof byte);

      // tell Arduino to stop sending data
      retval = write(fd, "T", 1);

      // drain garbage
      usleep(100000);
      tcflush(fd, TCIFLUSH);

      exit(0);

    default:
      close(pipe_fds[0]);
    }
  }

  bool PowerSensor::Sensor::inUse() const
  {
    return volt != 0;
  }

  void PowerSensor::Sensor::updateLevel(int16_t level)
  {
    double now = omp_get_wtime();

    wattAtlastMeasurement = -(level - 776) * weight - nullLevel;
    consumedEnergy += wattAtlastMeasurement * (now - timeAtLastMeasurement);
    timeAtLastMeasurement = now;
  }

  double PowerSensor::Sensor::totalEnergy(double now) const
  {
    return /* weight == 0 ? 0 : */ consumedEnergy + wattAtlastMeasurement * (now - timeAtLastMeasurement);
  }

  double PowerSensor::Sensor::currentWatt() const
  {
    return /* weight == 0 ? 0 : */ wattAtlastMeasurement;
  }

  bool PowerSensor::readLevelFromDevice(unsigned &sensorNumber, unsigned &level, unsigned &marker)
  {
    // two 8-bit integer buffers, currently to store the whole ADC DR of 32 bits (needs to be downscaled for writing performance);
    uint8_t buffer[2];

    // return value storage and bytesRead counter;
    uint8_t returnValue, bytesRead = 0;
    //std::cout << 'S';
    while (true)
    {
      //std::cout << 'W';
      // read N amount and save in the buffer, N is determined by subtracting amount of bytes it already received from the total buffer size expected;
      if ((returnValue = ::read(fd, (char *)&buffer + bytesRead, sizeof buffer - bytesRead)) < 0)
      {
        perror("read device");
        exit(1);
      }
      // if the amount of bytes it received is equal to the amount it expected, also checks if the return value of read is 0;
      else if ((bytesRead += returnValue) == sizeof buffer) //if ((bytesRead += returnValue) == sizeof buffer)
      {
        // if the received corresponds to kill signal, return false to terminate the IOthread;
        if (buffer[0] == 0xFF && buffer[1] == 0x3F)
        {
          std::cout << 'D' << std::endl;
          return false;
        }
        // checks if first byte corresponds with predetermined first byte format;
        else if ((buffer[0] & 0x80) &&
                 ((buffer[1] & 0x80) == 0))
        {
          //std::cout << 'G';
          //countera++;

          // extracts sensor number;
          sensorNumber = (buffer[0] >> 4) & 0x7;

          // extracts the level from the buffers;
          level = ((buffer[0] & 0xF) << 6) | (buffer[1] & 0x3F);

          // checks if there is a marker present;
          marker = (buffer[1] >> 6) & 0x1;
          return true;
        }
        else
        {
          //counterb++;
	  std::cout << "lost" << std::endl;
          // if a byte is lost, drop the first byte and try again;
          buffer[0] = buffer[1];

          bytesRead = 1;
        }
      }
    }
  }

  void PowerSensor::dumpCurrentWattToFile()
  {
    std::unique_lock<std::mutex> lock(dumpFileMutex);
    double totalWatt = 0;
    double time = omp_get_wtime();

    *dumpFile << "S " << time - startTime;

#if 1
    static double previousTime;
    *dumpFile << ' ' << 1e6 * (time - previousTime);
    previousTime = time;
#endif

    for (const Sensor &sensor : sensors)
      if (sensor.inUse())
      {
        *dumpFile << ' ' << sensor.currentWatt();
        totalWatt += sensor.currentWatt();
      }

    *dumpFile << ' ' << totalWatt << std::endl;
  }

  void PowerSensor::IOthread()
  {
    threadStarted.up();

    unsigned sensorNumber, level, marker;

    while (readLevelFromDevice(sensorNumber, level, marker))
    {
      std::unique_lock<std::mutex> lock(mutex);
      sensors[sensorNumber].updateLevel(level);

      if (dumpFile != nullptr)
        dumpCurrentWattToFile();
    }
    std::cout << "stopping io tred" << std::endl;
  }

  void PowerSensor::startIOthread()
  {
    if (thread == nullptr)
    {
      thread = new std::thread(&PowerSensor::IOthread, this);

      if (write(fd, "S", 1) != 1)
      {
        perror("write device");
        exit(1);
      }
    }

    threadStarted.down(); // wait for the IOthread to run smoothly
  }

  void PowerSensor::stopIOthread()
  {
    if (thread != nullptr)
    {
      if (write(fd, "X", 1) < 0)
      {
        perror("write");
        exit(1);
      }

      thread->join();
      delete thread;
      thread = 0;
    }
  }

  void PowerSensor::dump(const char *dumpFileName)
  {
    dumpFile = std::unique_ptr<std::ofstream>(dumpFileName != nullptr ? new std::ofstream(dumpFileName) : nullptr);
  }

  void PowerSensor::mark(const char *name) const
  {
    if (write(fd, "M", 1) < 0)
    {
      perror("write");
      exit(1);
    }
  }

  void PowerSensor::mark(const State &startState, const State &stopState, const char *name, unsigned tag) const
  {
    if (dumpFile != nullptr)
    {
      std::unique_lock<std::mutex> lock(dumpFileMutex);
      *dumpFile << "M " << startState.timeAtRead - startTime << ' ' << stopState.timeAtRead << ' ' << tag << " \"" << (name == nullptr ? "" : name) << '"' << std::endl;
    }
  }

  State PowerSensor::read() const
  {
    State state;

    std::unique_lock<std::mutex> lock(mutex);
    state.timeAtRead = omp_get_wtime();

    for (unsigned sensorID = 0; sensorID < MAX_SENSORS; sensorID++)
      state.consumedEnergy[sensorID] = sensors[sensorID].totalEnergy(state.timeAtRead);

    return state;
  }

  double Joules(const State &firstState, const State &secondState, int sensorID)
  {
    if (sensorID >= (signed)MAX_SENSORS)
      return 0;

    if (sensorID >= 0)
      return secondState.consumedEnergy[sensorID] - firstState.consumedEnergy[sensorID];

    double joules = 0;

    for (double consumedEnergy : secondState.consumedEnergy)
      joules += consumedEnergy;

    for (double consumedEnergy : firstState.consumedEnergy)
      joules -= consumedEnergy;

    return joules;
  }

  double seconds(const State &firstState, const State &secondState)
  {
    return secondState.timeAtRead - firstState.timeAtRead;
  }

  double Watt(const State &firstState, const State &secondState, int sensorID)
  {
    return Joules(firstState, secondState, sensorID) / seconds(firstState, secondState);
  }

  float PowerSensor::getVolt(unsigned sensorID) const
  {
    return sensorID < MAX_SENSORS ? sensors[sensorID].volt : 0;
  }

  void PowerSensor::setVolt(unsigned sensorID, float volt)
  {
    if (sensorID < MAX_SENSORS)
    {
      sensors[sensorID].setVolt(volt);
      writeSensorsToEEPROM();
    }
  }

  float PowerSensor::getType(unsigned sensorID) const
  {
    return sensorID < MAX_SENSORS ? sensors[sensorID].type : 0;
  }

  void PowerSensor::setType(unsigned sensorID, float type)
  {
    if (sensorID < MAX_SENSORS)
    {
      sensors[sensorID].setType(type);
      writeSensorsToEEPROM();
    }
  }

  float PowerSensor::getNullLevel(unsigned sensorID) const
  {
    return sensorID < MAX_SENSORS ? sensors[sensorID].nullLevel : 0;
  }

  void PowerSensor::setNullLevel(unsigned sensorID, float nullLevel)
  {
    if (sensorID < MAX_SENSORS)
    {
      sensors[sensorID].setNullLevel(nullLevel);
      writeSensorsToEEPROM();
    }
  }

  bool PowerSensor::inUse(unsigned sensorID) const
  {
    return sensorID < MAX_SENSORS && sensors[sensorID].inUse();
  }

} // namespace PowerSensor
