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
#include <sys/wait.h>
#include <unistd.h>


void usage(char *argv[])
{
  std::cerr << "usage: " << argv[0] << " [-d device] [-f dump_file] [-s sensor] -- command [args]" << std::endl;
  exit(1);
}


int main(int argc, char *argv[])
{
  const char *device = "/dev/ttyACM0", *dumpFileName = 0;
  int        sensor  = -1;

  for (int opt; (opt = getopt(argc, argv, "d:f:s:")) >= 0;) {
    switch (opt) {
      case 'd': device = optarg;
		break;

      case 'f': dumpFileName = optarg;
		break;

      case 's': sensor = atoi(optarg);
		break;

      default:  usage(argv);
    }
  }

  if (optind >= argc)
    usage(argv);

  PowerSensor::PowerSensor powerSensor(device);
  powerSensor.dump(dumpFileName);
  PowerSensor::State startState = powerSensor.read();
  int retval;

  switch (fork()) {
    case -1: perror("fork");
	     exit(1);

    case  0: execvp(argv[optind], argv + optind);
	     perror("execvp");
	     exit(1);

    default: if (wait(&retval) < 0) {
	       perror("wait");
	       exit(1);
	     }
  }

  PowerSensor::State stopState = powerSensor.read();

  std::cout <<
    PowerSensor::seconds(startState, stopState) << " s, " <<
    PowerSensor::Joules(startState, stopState, sensor) << " J, " <<
    PowerSensor::Watt(startState, stopState, sensor) << " W" <<
    std::endl;

  return retval;
}
