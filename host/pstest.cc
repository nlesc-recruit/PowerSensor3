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

#include <inttypes.h>
#include <unistd.h>

#define MAX_MICRO_SECONDS 4000000


void usage(char *argv[])
{
  std::cerr << "usage: " << argv[0] << " [-d device] [-f dump_file] [-s sensor]" << std::endl;
  exit(1);
}


int main(int argc, char *argv[])
{
  const char *device = "/dev/ttyACM1", *dumpFileName = 0;
  int        sensor  = -1;

  for (int opt; (opt = getopt(argc, argv, "d:f:s:")) >= 0;) {
    switch (opt) {
      case 'd': device = optarg;
		break;

      case 'f': dumpFileName = optarg;
		break;

      case 's': sensor = atoi(optarg);
		break;

      default:	usage(argv);
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
      PowerSensor::Joules(states[i ^ 1], states[i], sensor) << " J, " <<
      PowerSensor::Watt(states[i ^ 1], states[i], sensor) << " W" <<
      std::endl;
  }
  
  return 0;
}
