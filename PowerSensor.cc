#include "PowerSensor.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <omp.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <byteswap.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <sys/types.h>



namespace PowerSensor {

    PowerSensor::PowerSensor(const char *device)
    :
        fd(openDevice(device)),
        thread(nullptr) 
    {
        //startCleanupProcess(); // no clue what this is actually doing at the initialization of the program
        //readSensorsFromEEPROM(); // no EEPROM in the STM32F407 so this will need to be done some other way
      startIOthread();
    }

    PowerSensor::~PowerSensor()
    {
        //stopIOthread();
        if (close(fd))
            perror("close device");
    }

    bool PowerSensor::readLevelFromDevice(unsigned &sensorNumber, unsigned &level)
    {
      // two 8-bit integer buffers, currently to store the whole ADC DR of 32 bits (needs to be downscaled for writing performance);
      uint8_t buffer[2]; 

      // return value storage and bytesRead counter;
      uint8_t returnValue, bytesRead = 0;

      while (true) 
      {
        // read N amount and save in the buffer, N is determined by subtracting amount of bytes it already received from the total buffer size expected;
        if ((returnValue = ::read(fd, &buffer, sizeof(buffer))) < 0)
        {
          perror("read device");
          exit(1);
        }
        // if the amount of bytes it received is equal to the amount it expected, also checks if the return value of read is 0;
        else //if ((bytesRead += returnValue) == sizeof buffer)
        {
          // reconstruct the uint from individual bytes;
          level = buffer[0] << 8 | buffer[1];
          return true;
        }
      }
    }

    // constantly reads data from the device and saves it;
    void PowerSensor::IOthread()
    {
      // signal that the thread is running
      threadStarted.up();

      unsigned sensorNumber; // not in the protocol yet
      uint32_t level; // too large, scale down?

      while (readLevelFromDevice(sensorNumber, level))
      {
        if(dumpFile != nullptr) 
        {
          *dumpFile << level << std::endl;
        }
      }
    }

    // starts the IO thread;
    void PowerSensor::startIOthread() 
    {
      // if no thread exists, create one;
      if (thread == nullptr)
      {
        thread = new std::thread(&PowerSensor::IOthread, this);
        // write start character S to device;
        if (write(fd, "S", 1) != 1) 
        {
          perror("write device");
          exit(1);
        }
      }

      // wait for the IOthread to run smoothly;
      threadStarted.down();
    }

    // stops the IO thread;
    void PowerSensor::stopIOthread()
    {
      // check if there is actually a thread running;
      if (thread != nullptr)
      {
        // write stop command to device;
        if (write(fd, "X", 1) < 0) 
        {
          perror("write");
          exit(1);
        }

        // blocks the calling thread until the thread terminates;
        thread->join();

        // terminate and delete the thread instance;
        delete thread;

        // reset thread value for reassignment;
        thread = 0;
      }
    }

    // opens the device in parameter, returns a file descriptor of the device;
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
        cfsetispeed(&terminalOptions, B2000000);

        // sets the output baud rate;
        cfsetospeed(&terminalOptions, B2000000);

        // set control mode flags;
        terminalOptions.c_cflag |= (terminalOptions.c_cflag & ~CSIZE) | CS8;
        terminalOptions.c_cflag |= CLOCAL | CREAD;
        terminalOptions.c_cflag |= (PARENB | PARODD);

        // set input mode flags;
        terminalOptions.c_iflag |= IGNBRK;
        terminalOptions.c_iflag |=(IXON | IXOFF | IXANY);

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

    // enables outputting in a dumpfile and set its name;
    void PowerSensor::dump(const char *dumpFileName) 
    {
      dumpFile = std::unique_ptr<std::ofstream>(dumpFileName != nullptr ? new std::ofstream(dumpFileName) : nullptr);
    }

    // sends a marker to the device to mark accurately in the output file;
    void PowerSensor::mark() const
    {
      // writes M for marker;
      if (write(fd, "M", 1) != 1) 
      {
        perror("write device");
        exit(1);
      }
    }

}