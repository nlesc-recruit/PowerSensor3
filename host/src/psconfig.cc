#include "PowerSensor.hpp"

#include <unistd.h>

#include <cstring>
#include <iostream>
#include <memory>


std::unique_ptr<PowerSensor::PowerSensor> powerSensor;
unsigned int sensor = 0;


unsigned int selectSensor(unsigned int sensor) {
  if (sensor >= PowerSensor::MAX_SENSORS) {
    std::cerr << "Invalid sensor ID: " << sensor << ", max value is " << PowerSensor::MAX_SENSORS - 1 << std::endl;
  }
  return sensor;
}


float convertType(const char *arg)
{
  if (strcmp(arg, "ACS712-5") == 0 || strcmp(arg, "acs712-5") == 0)
    return 0.185;
  else if (strcmp(arg, "ACS712-20") == 0 || strcmp(arg, "acs712-20") == 0)
    return 0.1;
  else if (strcmp(arg, "ACS712-30") == 0 || strcmp(arg, "acs712-30") == 0)
    return 0.66;
  else
    return atof(arg);
}


PowerSensor::PowerSensor *checkPowerSensor()
{
  if (powerSensor.get() == nullptr)
    powerSensor = std::unique_ptr<PowerSensor::PowerSensor>(new PowerSensor::PowerSensor("/dev/ttyACM0"));

  return powerSensor.get();
}


void measureSensors(PowerSensor::State &startState, PowerSensor::State &stopState)
{
  startState = powerSensor->read();
  sleep(2);
  stopState = powerSensor->read();
}


// void autoCalibrate()
// {
//   if (powerSensor->inUse(sensor)) {
//     PowerSensor::State startState, stopState;

//     powerSensor->setNullLevel(sensor, 0);
//     measureSensors(startState, stopState);
//     powerSensor->setNullLevel(sensor, PowerSensor::Watt(startState, stopState, sensor));
//   }
// }


void print()
{
  PowerSensor::State startState, stopState;

  measureSensors(startState, stopState);

  char type[16];
  for (unsigned sensor = 0; sensor < PowerSensor::MAX_SENSORS; sensor++) {
    powerSensor->getType(sensor, type);
    std::cout << "sensor: " << sensor << ", "
      "Pair ID: " << (int) powerSensor->getPairId(sensor) << ", "
      "type: " << type << ", "
      "Vref: " << powerSensor->getVref(sensor) << " V, "
      "Slope: " << powerSensor->getSlope(sensor) << " V/A, " // TODO: is V/A the correct unit?
      "Status: " << (powerSensor->getInUse(sensor) ? "on" : "off") << std::endl;

    if (sensor % 2 == 1) {
      unsigned int pair = sensor / 2;
      std::cout << "Current usage pair " << pair << ": " << Watt(startState, stopState, pair) << " W" << std::endl;
    }
  }
}


void usage(char *argv[])
{
  std::cerr << "usage: " << argv[0] << " [-h] [-d device] [-s sensor] [-t type] [-v volt] [-a | -n nullLevel] [-o] [-p]" << std::endl;
  std::cerr << "-h prints this help" << std::endl;
  std::cerr << "-d selects the device (default: /dev/ttyACM0)" << std::endl;
  std::cerr << "-s selects the sensor (0-" << PowerSensor::MAX_SENSORS << ")" << std::endl;
  std::cerr << "-i sets the sensor pair (0-" << PowerSensor::MAX_PAIRS << ")" << std::endl;
  std::cerr << "-t sets the sensor type, (valid types TBD)" << std::endl;
  std::cerr << "-v sets the reference voltage level" << std::endl;
  std::cerr << "-n set the slope" << std::endl;
  // std::cerr << "-a auto-calibrates the null level" << std::endl;  // disabled while psconfig is still WIP
  std::cerr << "-o turns a sensor on (1) or off (0)" << std::endl;
  std::cerr << "-p prints configured values" << std::endl;
  // std::cerr << "example: " << argv[0] << " -d/dev/ttyACM0 -s0 -tACS712-20 -v12 -a -s1 -tACS712-5 -v3.3 -a -s2 -tACS712-20 -v12 -a -s3 -o -s4 -o -p" << std::endl;
  // exit(1);
}


int main(int argc, char *argv[])
{
  for (int opt; (opt = getopt(argc, argv, "d:s:i:t:v:n:o:ph")) >= 0;) {
    switch (opt) {
      // device select
      case 'd':
        powerSensor = std::unique_ptr<PowerSensor::PowerSensor>(new PowerSensor::PowerSensor(optarg));
    		break;

      // sensor select
      case 's':
        sensor = selectSensor((unsigned) atoi(optarg));
        break;

      // sensor pair
      case 'i':
        checkPowerSensor()->setPairId(sensor, atoi(optarg));
        break;

      // sensor type
      case 't':
        checkPowerSensor()->setType(sensor, optarg);
        break;

      // sensor reference voltage
      case 'v':
        checkPowerSensor()->setVref(sensor, atof(optarg));
        break;

      // sensor slope
      case 'n':
        checkPowerSensor()->setSlope(sensor, atof(optarg));
        break;

      // sensor on/off
      case 'o':
        checkPowerSensor()->setInUse(sensor, bool(optarg));
        break;

      // print
      case 'p':
        print();
        break;

      // help
      case 'h':
        usage(argv);
        break;

      default:
        usage(argv);
        break;
    }
  }

  if ((optind < argc) || (argc < 2))
    usage(argv);

  return 0;
}
