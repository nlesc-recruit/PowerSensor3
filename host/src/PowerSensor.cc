#include "PowerSensor.h"

#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// 4M baudrate is not defined by default on Mac
#ifdef __APPLE__
#define B4000000 0010017
#endif

namespace PowerSensor {

  PowerSensor::PowerSensor(const char* device): fd(openDevice(device)) {};

  PowerSensor::~PowerSensor() {close(fd);};

  int PowerSensor::openDevice(const char* device) {
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

    // set input mode flags;
    terminalOptions.c_iflag = 0;
    //terminalOptions.c_iflag |= IGNBRK;

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

} // namespace PowerSensor