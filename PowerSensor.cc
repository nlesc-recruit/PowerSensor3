#include "PowerSensor.h"

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
#include <sys/types.h>



namespace PowerSensor {

    PowerSensor::PowerSensor(const char *device)
    :
        fd(openDevice(device)),
        thread(nullptr) 
    {
      std::cout << "Constructor" << std::endl;
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
      // four 8-bit integer buffers, currently to store the whole ADC DR of 32 bits (needs to be downscaled for writing performance);
      uint8_t buffer[4]; 

      // return value storage and bytesRead counter;
      uint8_t returnValue, bytesRead = 0;

      while (true) 
      {
        std::cout << "Reading..." << std::endl;
        // read N amount and save in the buffer, N is determined by subtracting amount of bytes it already received from the total buffer size expected;
        if ((returnValue = ::read(fd, (char *) &buffer + bytesRead, sizeof buffer - bytesRead)) < 0)
        {
          perror("read device");
          exit(1);
        }
        // if the amount of bytes it received is equal to the amount it expected, also checks if the return value of read is 0;
        else //if ((bytesRead += returnValue) == sizeof buffer)
        {
          std::cout << "Receiving..." << std::endl;
          // reconstruct the uint32 from individual bytes;
          level = buffer[0] >> 24 | buffer[1] >> 16 | buffer[2] >> 8 | buffer[3];
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

      std::cout << "Reading from device" << std::endl;
      while (readLevelFromDevice(sensorNumber, level))
      {
        std::cout << level << std::endl;
      }
    }

    // starts an IO thread;
    void PowerSensor::startIOthread() 
    {
      // if no thread exists, create one;
      if (thread == nullptr)
      {
        std::cout << "Creating thread" << std::endl;
        thread = new std::thread(&PowerSensor::IOthread, this);

        // write start character S to device;
        //if (write(fd, "S", 1) != 1) 
        //{
        //  perror("write device");
        //  exit(1);
        //}
      }

      // wait for the IOthread to run smoothly;
      std::cout << "Waiting for IOthread to run smoothly" << std::endl;
      threadStarted.down();
      std::cout << "IOthread runs smoothly" << std::endl;
    }

    // opens the device in parameter, returns a file descriptor of the device;
    int PowerSensor::openDevice(const char *device)
    {
        int fileDescriptor; 
  
        std::cout << "Opening Device" << std::endl;
        // opens the file specified by pathname;
        if ((fileDescriptor = open(device, O_RDWR)) < 0)
        {
            perror("open device");
            exit(1);
        }
        std::cout << "Device opened" << std::endl;
        // block if an incompatible lock is held by another process;
        if (flock(fileDescriptor, LOCK_EX) < 0)
        {
            perror("flock");
            exit(1);
        }

        std::cout << "Configuring terminal" << std::endl;
        // struct for configuring the port for communication with stm32;
        struct termios terminalOptions;

        // gets the current options for the port;
        tcgetattr(fileDescriptor, &terminalOptions);

        // sets the input baud rate;
        cfsetispeed(&terminalOptions, B9600);

        // sets the output baud rate;
        cfsetospeed(&terminalOptions, B9600);

        // set control mode flags;
        terminalOptions.c_cflag = (terminalOptions.c_cflag & ~CSIZE) | CS8;
        terminalOptions.c_cflag |= CLOCAL | CREAD;
        terminalOptions.c_cflag &= ~(PARENB | PARODD);

        // set input mode flags;
        terminalOptions.c_iflag = IGNBRK;
        terminalOptions.c_iflag &= ~(IXON | IXOFF | IXANY);

        // clear local mode flag
        terminalOptions.c_lflag = 0;

        // clear output mode flag;
        terminalOptions.c_oflag = 0;

        // set control characters;
        terminalOptions.c_cc[VMIN] = 2;
        terminalOptions.c_cc[VTIME] = 0;

        std::cout << "Commiting terminal configurations" << std::endl;
        // commit the options;
        tcsetattr(fileDescriptor, TCSANOW, &terminalOptions);

        std::cout << "Flushing serial buffer" << std::endl;
        // flush anything already in the serial buffer;
        tcflush(fileDescriptor, TCIFLUSH);

        return fileDescriptor;
    }


}