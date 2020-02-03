#include "PowerSensor.h"

#include <unistd.h>

#include <cstring>
#include <iostream>
#include <memory>


std::unique_ptr<PowerSensor::PowerSensor> powerSensor;
int sensor = 0;


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


void autoCalibrate()
{
  if (powerSensor->inUse(sensor)) {
    PowerSensor::State startState, stopState;

    powerSensor->setNullLevel(sensor, 0);
    measureSensors(startState, stopState);
    powerSensor->setNullLevel(sensor, PowerSensor::Watt(startState, stopState, sensor));
  }
}


bool approximates(float a, float b)
{
  return a / b > .999999 && a / b < 1.000001;
}


std::string type(unsigned sensor)
{
  float voltPerAmpere = powerSensor->getType(sensor);

  if (approximates(voltPerAmpere, .185))
    return "ACS712-5";
  else if (approximates(voltPerAmpere, .1))
    return "ACS712-20";
  else if (approximates(voltPerAmpere, .66))
    return "ACS712-30";
  else
    return std::to_string(voltPerAmpere) + " V/A";
}


void print()
{
  PowerSensor::State startState, stopState;

  measureSensors(startState, stopState);

  for (unsigned sensor = 0; sensor < PowerSensor::MAX_SENSORS; sensor ++)
    if (powerSensor->inUse(sensor))
      std::cout << "sensor: " << sensor << ", "
		   "volt: " << powerSensor->getVolt(sensor) << " V, "
		   "type: " << type(sensor) << ", "
		   "null level: " << powerSensor->getNullLevel(sensor) << " W, "
		   "current usage: " << Watt(startState, stopState, sensor) << " W" << std::endl;
    else
      std::cout << "sensor: " << sensor << ", off" << std::endl;
}


void usage(char *argv[])
{ 
  std::cerr << "usage: " << argv[0] << " [-d device] [-s sensor] [-t type] [-v volt] [-a | -n nullLevel] [-o] [-p]" << std::endl;
  std::cerr << "-d selects the device (default: /dev/ttyACM0)" << std::endl;
  std::cerr << "-s selects the sensor (0-4)" << std::endl;
  std::cerr << "-t selects the sensor type, (one of ACS712-5, ACS712-20, ACS712-30, or a Volt-Per-Ampere value)" << std::endl;
  std::cerr << "-v sets the voltage level" << std::endl;
  std::cerr << "-a auto-calibrates the null level" << std::endl;
  std::cerr << "-n manually sets the null level (the amount of Watt to be added)" << std::endl;
  std::cerr << "-o turns off a sensor" << std::endl;
  std::cerr << "-p prints configured values" << std::endl;
  std::cerr << "example: " << argv[0] << " -d/dev/ttyACM0 -s0 -tACS712-20 -v12 -a -s1 -tACS712-5 -v3.3 -a -s2 -tACS712-20 -v12 -a -s3 -o -s4 -o -p" << std::endl;
  exit(1);
}


int main(int argc, char *argv[])
{
  for (int opt; (opt = getopt(argc, argv, "ad:n:ops:t:v:")) >= 0;) {
    switch (opt) {
      case 'a': checkPowerSensor();
		autoCalibrate();
		break;

      case 'd':	powerSensor = std::unique_ptr<PowerSensor::PowerSensor>(new PowerSensor::PowerSensor(optarg));
		break;

      case 'n': checkPowerSensor()->setNullLevel(sensor, atof(optarg));
		break;

      case 'o':	checkPowerSensor()->setVolt(sensor, 0);
		break;

      case 'p': checkPowerSensor();
		print();
		break;

      case 's': sensor = atoi(optarg);
		break;

      case 't': checkPowerSensor()->setType(sensor, convertType(optarg));
		break;

      case 'v': checkPowerSensor()->setVolt(sensor, atof(optarg));
		break;

      default:	usage(argv);
    }
  }

  if (optind < argc)
    usage(argv);

  return 0;
}
