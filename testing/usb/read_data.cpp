#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

int openDevice(std::string device) {
  int fileDescriptor;

  // opens the file specified by pathname;
  if ((fileDescriptor = open(device.c_str(), O_RDWR)) < 0) {
    perror(device.c_str());
    exit(1);
  }
  // block if an incompatible lock is held by another process;
  if (flock(fileDescriptor, LOCK_EX) < 0) {
    perror("flock");
    exit(1);
  }

  // struct for configuring the port for communication with stm32;
  struct termios terminalOptions;

  // gets the current options for the port;
  tcgetattr(fileDescriptor, &terminalOptions);

  // set control mode flags;
  terminalOptions.c_cflag = (terminalOptions.c_cflag & ~CSIZE) | CS8;
  terminalOptions.c_cflag |= CLOCAL | CREAD;
  terminalOptions.c_cflag &= ~(PARENB | PARODD);


  // set input mode flags;
  terminalOptions.c_iflag |= IGNBRK;
  terminalOptions.c_iflag &= ~(IXON | IXOFF | IXANY);

  // clear local mode flag
  terminalOptions.c_lflag = 0;

  // clear output mode flag;
  terminalOptions.c_oflag = 0;

  // set control characters;
  terminalOptions.c_cc[VMIN] = 1;
  terminalOptions.c_cc[VTIME] = 0;

  // commit the options;
  tcsetattr(fileDescriptor, TCSANOW, &terminalOptions);

  // flush anything already in the serial buffer;
  tcflush(fileDescriptor, TCIFLUSH);

  return fileDescriptor;
  }

inline void writeCharToDevice(int fd, char buffer) {
  if (write(fd, &buffer, 1) != 1) {
      perror("write device");
  }
}

void readValueFromDevice(int fd, uint16_t *value, uint8_t *id) {
  // buffer for one 16b packet
  uint8_t buffer[2];
  unsigned int retVal, bytesRead = 0;
  // loop exits when valid value is read from device
  while (true) {
    // read full buffer
    do {
      if ((retVal = ::read(fd, reinterpret_cast<char*>(&buffer) + bytesRead, sizeof(buffer) - bytesRead)) < 0) {
        perror("read");
        exit(1);
      }
    } while ((bytesRead += retVal) < sizeof buffer);

    if ((buffer[0] >> 6) != (buffer[1] >> 6)) {
      // both bytes should be from the same sensor, if not drop the first byte and try again
      std::cerr << "Received bytes from 2 different sensors, dropping one and trying again" << std::endl;
      buffer[0] = buffer[1];
      bytesRead = 1;
    } else {
      // extract sensor ID and values
      *id = buffer[0] >> 6;
      *value = ((buffer[0] & 0x1F) << 5) | (buffer[1] & 0x1F);
      return;
    }
  }
}

void dataReader(int fd, std::string dumpFileName, bool* quit) {
  // write header
  std::ofstream dumpFile(dumpFileName);
  dumpFile << "#t    current    voltage    dt" << std::endl;

  uint16_t t, current, voltage;

  while (!(*quit)) {
    static unsigned counter = 0;
    uint8_t id;
    uint16_t value;
    readValueFromDevice(fd, &value, &id);
    switch (id) {
      case 0b11:
        t = value;
        counter++;
        break;
      case 0b00:
        current = value;
        counter++;
        break;
      case 0b01:
        voltage = value;
        counter++;
        break;
      default:
        std::cerr << "Unknown sensor ID: " << id << std::endl;
        break;
    }
    // if all three values have been updated, write to file and reset
    if (counter >= 3) {
      static uint16_t prev_t = t;
      uint16_t dt = ( t - prev_t );
      dt %= 1024;  // in two lines because % is only modulo for positive numbers
      dumpFile << t << " " << current << " " << voltage << " " << dt << std::endl;
      std::cerr << t << " " << current << " " << voltage << " " << dt << std::endl;
      counter = 0;
      prev_t = t;
    }
  }
  dumpFile.close();
}

int main() {
  std::string device = "/dev/cu.usbmodem207338A658481";
  // std::string device = "/dev/cu.usbmodem144203";
  std::string dumpFileName = "out.txt";
  bool quit = false;

  int fd = openDevice(device);
  std::cout << "Connected to device " << std::endl;

  // thread for reading data
  std::thread thread(dataReader, fd, dumpFileName, &quit);

  // start measurement
  writeCharToDevice(fd, 'S');
  // std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  // stop measurement
  writeCharToDevice(fd, 'T');
  quit = true;
  thread.join();

  if (close(fd)) {
    perror("close device");
  }
  std::cout << "Closed connection to device" << std::endl;
  return 0;
}
