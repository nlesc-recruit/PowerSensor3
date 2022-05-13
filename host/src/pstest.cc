#include <unistd.h>

#include <iostream>
#include <string>

#include "PowerSensor.hpp"


#define MAX_MICRO_SECONDS 4000000


void usage(char *argv[]) {
  std::cerr << "usage: " << argv[0] << " [-d device] [-f dump_file] [-s sensor_pair]" << std::endl;
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

  if (optind < argc)
    usage(argv);

  PowerSensor::PowerSensor powerSensor(device);
  powerSensor.dump(dumpFileName);

  PowerSensor::State states[2];
  states[0] = powerSensor.read();

  for (uint32_t micros = 100, i = 1; micros <= MAX_MICRO_SECONDS; micros *= 2, i ^= 1) {
    usleep(micros);
    states[i] = powerSensor.read();

    std::cout << "exp. time: " << micros * 1e-6 << " s, " "measured: " <<
      PowerSensor::seconds(states[i ^ 1], states[i]) << " s, " <<
      PowerSensor::Joules(states[i ^ 1], states[i], sensorPair) << " J, " <<
      PowerSensor::Watt(states[i ^ 1], states[i], sensorPair) << " W" <<
      std::endl;
  }

  return 0;
}
