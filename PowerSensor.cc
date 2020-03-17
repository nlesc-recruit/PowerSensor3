#include "PowerSensor.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <bitset>

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

  bool temprunning = true;

  uint32_t countera = 0, counterb = 0;

    PowerSensor::PowerSensor(const char *device)
    :
      fd(openDevice(device)),
      thread(nullptr) 
    {
      //startCleanupProcess(); // no clue what this is actually doing at the initialization of the program
      //readSensorsFromEEPROM(); // Emulated EEPROM is working so this can be implemented?
      startIOthread();
    }

    PowerSensor::~PowerSensor()
    {
      stopIOthread();
      std::cout << "No of samples: " << countera << std::endl;
      std::cout << "No of bytelosses: " << counterb << std::endl;
      if (close(fd))
        perror("close device");
    }

    void PowerSensor::readSensorsFromEEPROM() 
    {
      //stopIOthread();
      sleep(2);
      if (write(fd, "R", 1) != 1) 
      {
        perror("write device");
        exit(1);
      }
      std::cout << "readSensorsFromEEPROM()" << std::endl;
      struct EEPROM eeprom;
      ssize_t retval, bytesRead = 0;

      do {
        if ((retval = ::read(fd, (char *) &eeprom + bytesRead, sizeof eeprom - bytesRead)) < 0) {
          perror("read device");
          exit(1);
        }
      } while ((bytesRead += retval) < sizeof eeprom);
      
      for (int i = 0; i < MAX_SENSORS; i++)
      {
        std::cout << "Sensor: " << i << std::endl;
        std::cout << eeprom.sensors[i].type << std::endl;
        std::cout << eeprom.sensors[i].volt << std::endl;
        std::cout << eeprom.sensors[i].nullLevel << std::endl;
      }
    }

    void PowerSensor::writeSensorsToEEPROM()
    {
      if (write(fd, "W", 1) != 1) 
      {
        perror("write device");
        exit(1);
      }
      std::cout << "writeSensorsToEEPROM()" << std::endl;
      struct EEPROM eeprom;
      for (int i = 0; i < MAX_SENSORS; i++)
      {
        eeprom.sensors[i].type = .6;
        eeprom.sensors[i].volt = 3.3;
        eeprom.sensors[i].nullLevel = i;
      }
      ssize_t retval, bytesWritten = 0;
      do
      {
        if ((retval = ::write(fd, (char *) &eeprom + bytesWritten, sizeof eeprom - bytesWritten)) < 0) {
          perror("write device");
          exit(1);
        }
      } while ((bytesWritten += retval) < sizeof eeprom);
      
      
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
        if ((returnValue = ::read(fd, (char *) &buffer + bytesRead, sizeof buffer - bytesRead)) < 0)
        {
          perror("read device");
          exit(1);
        }
        // if the amount of bytes it received is equal to the amount it expected, also checks if the return value of read is 0;
        else if ((bytesRead += returnValue) == sizeof buffer) //if ((bytesRead += returnValue) == sizeof buffer)
        {
          // if the received corresponds to kill signal, return false to terminate the IOthread;
          if (buffer[0] == 0xFF && buffer[1] == 0xE0)
          {
            std::cout << 'D' << std::endl;
            return false;
          }
          // checks if first byte corresponds with predetermined first byte format;
          else if ((buffer[0] & 0x80) &&
          ((buffer[1] & 0x80) == 0))                    
          {
            //std::cout << 'G';
            countera++;

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
            counterb++;

            // if a byte is lost, drop the first byte and try again;
            buffer[0] = buffer[1];

            bytesRead = 1;
          }
        } 
      }
    }

    void PowerSensor::Sensor::updateLevel(int16_t level)
    {
      // get the current time;
      //double now = omp_get_wtime();

      //wattAtlastMeasurement = (level - 512) * weight - nullLevel;

      //consumedEnergy += wattAtlastMeasurement * (now - timeAtLastMeasurement);

      //timeAtLastMeasurement = now;
    }

    // constantly reads data from the device and saves it;
    void PowerSensor::IOthread()
    {
      // signal that the thread is running
      threadStarted.up();

      unsigned sensorNumber, level, marker; 

      while (readLevelFromDevice(sensorNumber, level, marker))
      {
        std::unique_lock<std::mutex> lock(mutex);
        sensors[sensorNumber].updateLevel(level); //.updateLevel(level);

        float volt = ((volt = level) / 512) * 1.65;
        float amp = ((volt - 1.65) / .185);

        if(dumpFile != nullptr) 
        {
          if (marker) 
            *dumpFile << 'M' << std::endl;

          *dumpFile <<  level << std::endl;
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
        cfsetispeed(&terminalOptions, B4000000);

        // sets the output baud rate;
        cfsetospeed(&terminalOptions, B4000000);

        // set control mode flags;
        terminalOptions.c_cflag |=  CLOCAL | CREAD | CS8;
        //terminalOptions.c_cflag |= (PARENB | PARODD);

        // set input mode flags;
        terminalOptions.c_iflag = 0;
        //terminalOptions.c_iflag |= IGNBRK;
        //terminalOptions.c_iflag |=(IXON | IXOFF | IXANY);

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
