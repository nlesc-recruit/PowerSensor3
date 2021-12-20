//  Copyright (C) 2016
//  ASTRON (Netherlands Institute for Radio Astronomy) / John W. Romein
//  P.O. Box 2, 7990 AA  Dwingeloo, the Netherlands

//  This file is part of PowerSensor.

//  PowerSensor is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.

//  PowerSensor is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with PowerSensor.  If not, see <http://www.gnu.org/licenses/>.


#include "PowerSensor.h"

#include <iostream>
#include <iomanip>
#include <cmath>

#include <inttypes.h>
#include <unistd.h>

#define MAX_MICRO_SECONDS 4000000


void usage(char *argv[])
{
  std::cerr << "usage: " << argv[0] << " [-d device] [-s sensor] [-i milliseconds] [-b nbit] [-v voltage]" << std::endl;
  exit(1);
}


int main(int argc, char *argv[])
{
  const char *device = "/dev/ttyACM1";
  int interval = 100 * 1000;
  int sensor;
  int ADCmax = pow(2, 10);
  float maxVoltage = 3.3;

  bool sets = false;

  for (int opt; (opt = getopt(argc, argv, "d:i:s:b:v:")) >= 0;) {
    switch (opt) {
      case 'd':
        device = optarg;
		break;

      case 'i':
        interval = 1000 * atoi(optarg); // convert to microseconds
		break;

      case 's':
        sensor = atoi(optarg);
        sets = true;
		break;

      case 'b':
        ADCmax = pow(2, atoi(optarg));
        break;

      case 'v':
        maxVoltage = atof(optarg);
        break;

      default:	usage(argv);
    }
  }

  if (!sets)
    usage(argv);

  PowerSensor::PowerSensor powerSensor(device);

  while (true) {
      if (sensor == -1) {
        // print values for all sensors
        std::cout << std::fixed << std::setprecision(2);
        for (int s = 0; s < PowerSensor::MAX_SENSORS; s++)
        {
          std::cout << s << " " << (powerSensor.getRawLevel(s) * maxVoltage) / ADCmax << std::endl;
        }
        std::cout << std::endl;

      } else {
        // print value for single sensor
        std::cout << std::fixed << std::setprecision(2) <<
          (powerSensor.getRawLevel(sensor) * maxVoltage) / ADCmax << std::endl;
      }

      usleep(interval);
  }

  return 0;
}
