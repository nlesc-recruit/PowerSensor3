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


namespace PowerSensor {


void PowerSensor::Sensor::readFromEEPROM(int fd)
{
  struct EEPROM eeprom;
  ssize_t       retval, bytesRead = 0;

  do {
    if ((retval = ::read(fd, (char *) &eeprom + bytesRead, sizeof eeprom - bytesRead)) < 0) {
      perror("read device");
      exit(1);
    }
  } while ((bytesRead += retval) < sizeof eeprom);

#if defined __BIG_ENDIAN__
  eeprom.volt      = __bswap_32(eeprom.volt);
  eeprom.type      = __bswap_32(eeprom.type);
  eeprom.nullLevel = __bswap_32(eeprom.nullLevel);
#endif

  setVolt(eeprom.volt);
  setType(eeprom.type);
  setNullLevel(eeprom.nullLevel);
}


void PowerSensor::Sensor::writeToEEPROM(int fd) const
{
  struct EEPROM eeprom;

  eeprom.volt	   = volt;
  eeprom.type      = type;
  eeprom.nullLevel = nullLevel;

#if defined __BIG_ENDIAN__
  eeprom.volt      = __bswap_32(eeprom.volt);
  eeprom.type      = __bswap_32(eeprom.type);
  eeprom.nullLevel = __bswap_32(eeprom.nullLevel);
#endif

  ssize_t retval, bytesWritten = 0;

  do {
    if ((retval = ::write(fd, (char *) &eeprom + bytesWritten, sizeof eeprom - bytesWritten)) < 0) {
      perror("write device");
      exit(1);
    }
  } while ((bytesWritten += retval) < sizeof eeprom);
}


void PowerSensor::Sensor::updateDerivedValues()
{
  weight		= volt != 0 ? 2.5 / 512 * volt / type : 0;
  consumedEnergy	= 0;
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
  int fd;

  if ((fd = open(device, O_RDWR)) < 0) {
    perror("open device");
    exit(1);
  }

  if (flock(fd, LOCK_EX) < 0) {
    perror("flock");
    exit(1);
  }

  //Configure port for 8N1 transmission
  struct termios options;

  tcgetattr(fd, &options);		//Gets the current options for the port
  cfsetispeed(&options, B9600);	//Sets the Input Baud Rate
  cfsetospeed(&options, B9600);	//Sets the Output Baud Rate

  options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
  options.c_iflag = IGNBRK;
  options.c_lflag = 0;
  options.c_oflag = 0;
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cc[VMIN] = 2;
  options.c_cc[VTIME] = 0;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_cflag &= ~(PARENB | PARODD);

  /* commit the options */
  tcsetattr(fd, TCSANOW, &options);

#if defined UNO
  /* Wait for the Arduino to reset */
  sleep(2);
#endif

  /* Flush anything already in the serial buffer */
  tcflush(fd, TCIFLUSH);

  return fd;
}


PowerSensor::PowerSensor(const char *device)
:
  startTime(omp_get_wtime()),
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
  if (write(fd, "r", 1) != 1) {
    perror("write device");
    exit(1);
  }

  for (Sensor &sensor : sensors)
    sensor.readFromEEPROM(fd);
}


void PowerSensor::writeSensorsToEEPROM()
{
  stopIOthread();

  if (write(fd, "w", 1) != 1) {
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

  if (pipe(pipe_fds) < 0) {
    perror("pipe");
    exit(1);
  }

  switch (fork())
  {
    ssize_t retval;

    case -1 : perror("fork");
	      exit(1);

    case  0 : // detach from the parent process, so signals to the parent are not caught by the child
	      setsid();

	      // close all file descriptors, except pipe read end and Arduino fd
	      for (int i = 3, last = getdtablesize(); i < last; i ++)
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

    default:  close(pipe_fds[0]);
  }
}


bool PowerSensor::Sensor::inUse() const
{
  return volt != 0;
}


void PowerSensor::Sensor::updateLevel(int16_t level)
{
  double now = omp_get_wtime();

  wattAtlastMeasurement = (level - 512) * weight - nullLevel;
  consumedEnergy       += wattAtlastMeasurement * (now - timeAtLastMeasurement);
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


bool PowerSensor::readLevelFromDevice(unsigned &sensorNumber, unsigned &level)
{
  uint8_t msg[2];
  ssize_t retval, bytesRead = 0;

  while (true) {
    if ((retval = ::read(fd, (char *) &msg + bytesRead, sizeof msg - bytesRead)) < 0) {
      perror("read device");
      exit(1);
    } else if ((bytesRead += retval) == sizeof msg) {
      if (msg[0] == 0xFF && msg[1] == 0xE0) {
	      return false;
      } else if ((sensorNumber = msg[0] >> 5) < MAX_SENSORS && (msg[1] & 0xE0) == 0xE0) {
	      level = ((msg[0] & 0x1F) << 5) | (msg[1] & 0x1F);
	      return true;
      } else {
	      msg[0] = msg[1]; // byte lost?  drop first byte and try again
	      bytesRead = 1;
      }
    }
  }
}


void PowerSensor::dumpCurrentWattToFile()
{
  std::unique_lock<std::mutex> lock(dumpFileMutex);
  double totalWatt = 0;
  double time      = omp_get_wtime();

  *dumpFile << "S " << time - startTime;

#if 1
  static double previousTime;
  *dumpFile << ' ' << 1e6 * (time - previousTime);
  previousTime = time;
#endif

  for (const Sensor &sensor : sensors)
    if (sensor.inUse()) {
      *dumpFile << ' ' << sensor.currentWatt();
      totalWatt += sensor.currentWatt();
    }

  *dumpFile << ' ' << totalWatt << std::endl;
}


void PowerSensor::IOthread()
{
  threadStarted.up();

  unsigned sensorNumber, level;

  while (readLevelFromDevice(sensorNumber, level)) {
    std::unique_lock<std::mutex> lock(mutex);
    sensors[sensorNumber].updateLevel(level);

    if (dumpFile != nullptr)
      dumpCurrentWattToFile();
  }
}


void PowerSensor::startIOthread()
{
  if (thread == nullptr) {
    thread = new std::thread(&PowerSensor::IOthread, this);

    if (write(fd, "S", 1) != 1) {
      perror("write device");
      exit(1);
    }
  }

  threadStarted.down(); // wait for the IOthread to run smoothly
}


void PowerSensor::stopIOthread()
{
  if (thread != nullptr) {
    if (write(fd, "X", 1) < 0) {
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


void PowerSensor::mark(const State &state, const char *name, unsigned tag) const
{
  if (dumpFile != nullptr) {
    std::unique_lock<std::mutex> lock(dumpFileMutex);
    *dumpFile << "M " << state.timeAtRead - startTime << ' ' << tag << " \"" << (name == nullptr ? "" : name) << '"' << std::endl;
  }
}


void PowerSensor::mark(const State &startState, const State &stopState, const char *name, unsigned tag) const
{
  if (dumpFile != nullptr) {
    std::unique_lock<std::mutex> lock(dumpFileMutex);
    *dumpFile << "M " << startState.timeAtRead - startTime << ' ' << stopState.timeAtRead << ' ' << tag << " \"" << (name == nullptr ? "" : name) << '"' << std::endl;
  }
}


State PowerSensor::read() const
{
  State state;

  std::unique_lock<std::mutex> lock(mutex);
  state.timeAtRead = omp_get_wtime();

  for (unsigned sensorID = 0; sensorID < MAX_SENSORS; sensorID ++)
    state.consumedEnergy[sensorID] = sensors[sensorID].totalEnergy(state.timeAtRead);

  return state;
}


double Joules(const State &firstState, const State &secondState, int sensorID)
{
  if (sensorID >= (signed) MAX_SENSORS)
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
  if (sensorID < MAX_SENSORS) {
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
  if (sensorID < MAX_SENSORS) {
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
  if (sensorID < MAX_SENSORS) {
    sensors[sensorID].setNullLevel(nullLevel);
    writeSensorsToEEPROM();
  }
}


bool PowerSensor::inUse(unsigned sensorID) const
{
  return sensorID < MAX_SENSORS && sensors[sensorID].inUse();
}


}
