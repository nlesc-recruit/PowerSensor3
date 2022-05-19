#include <sys/wait.h>
#include <unistd.h>

#include <iostream>
#include <string>

#include "PowerSensor.hpp"


void usage(char *argv[]) {
  std::cerr << "usage: " << argv[0] << " [-d device] [-f dump_file] [-s sensor_pair] -- command [args]" << std::endl;
  exit(1);
}


int main(int argc, char *argv[]) {
  std::string device = "/dev/ttyACM1";
  std::string dumpFileName;
  int sensorPair  = -1;

  for (int opt; (opt = getopt(argc, argv, "d:f:s:")) >= 0;) {
    switch (opt) {
      case 'd':
        device = optarg;
        break;

      case 'f':
        dumpFileName = optarg;
        break;

      case 's':
        sensorPair = atoi(optarg);
        break;

      default:
        usage(argv);
        break;
    }
  }

  if (optind >= argc)
    usage(argv);

  PowerSensor::PowerSensor powerSensor(device);
  powerSensor.dump(dumpFileName);
  PowerSensor::State startState = powerSensor.read();
  int retval;

  switch (fork()) {
    case -1:
      perror("fork");
      exit(1);

    case 0:
      execvp(argv[optind], argv + optind);
      perror("execvp");
      exit(1);

    default:
      if (wait(&retval) < 0) {
        perror("wait");
        exit(1);
      }
      break;
  }

  PowerSensor::State stopState = powerSensor.read();

  std::cout <<
    PowerSensor::seconds(startState, stopState) << " s, " <<
    PowerSensor::Joules(startState, stopState, sensorPair) << " J, " <<
    PowerSensor::Watt(startState, stopState, sensorPair) << " W" <<
    std::endl;

  return retval;
}
