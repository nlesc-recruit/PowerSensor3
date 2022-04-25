#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif

#include <fcntl.h>
#include <termios.h>
#include <limits.h>
#include <unistd.h>

#include <thread>
#include <iostream>

#include "device.hpp"

namespace psemu = PowerSensorEmulator;

void setTerminalOptions(int fd) {
  struct termios terminalOptions;

  // gets the current options for the port;
  tcgetattr(fd, &terminalOptions);

  // use the same settings as when connecting to real PowerSensor device
  terminalOptions.c_cflag |= CLOCAL | CREAD | CS8;
  terminalOptions.c_iflag = 0;
  terminalOptions.c_iflag |= IGNCR;
  terminalOptions.c_lflag = 0;
  terminalOptions.c_oflag = 0;
  terminalOptions.c_cc[VMIN] = 0;
  terminalOptions.c_cc[VTIME] = 0;

  tcsetattr(fd, TCSANOW, &terminalOptions);
  fcntl(fd, F_SETFL, O_NONBLOCK);
}

void sigint_handler(sig_atomic_t s) {
  psemu::stop();  // stops the thread that writes data and the main serial event loop
  // rest of main handles exiting
}

int main() {
  signal(SIGINT, sigint_handler);

  // create pty for communication
  int fd_master, fd_slave;
  char device[PATH_MAX];
  if (openpty(&fd_master, &fd_slave, device, NULL, NULL) < 0) {
    perror("Failed to open pty");
    exit(1);
  }
  setTerminalOptions(fd_master);

  // separate thread for writing data
  std::thread* thread = new std::thread(&psemu::writeLoop, fd_master);

  std::cout << "PowerSensor emulator running at " << device << ", exit with ctrl+c." << std::endl;

  // main loop: receiving events
  psemu::serialEventLoop(fd_master);

  // when serialEventLoop returns, the program should cleanly exit
  // first join thet writeLoop thread
  if (thread != nullptr) {
    thread->join();
  }
  // close file descriptors
  if (close(fd_master) < 0)
    perror("Failed to close file descriptor");
  if (close(fd_slave) < 0)
    perror("Failed to close file descriptor");
  return 0;
}
