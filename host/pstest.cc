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
#include <bitset>

void shizzle() 
{

}


int main(int argc, char *argv[])
{
  const char *device = "/dev/ttyACM1";
  const char *dumpFileName = "output.txt";

  std::cout << "Setting up PowerSensor on port: " << device << std::endl;
  PowerSensor::PowerSensor powerSensor(device);
  std::cout << "PowerSensor up and running" << std::endl;
  powerSensor.dump(dumpFileName);
  std::cout << "Writing to: " << dumpFileName << std::endl;
  //powerSensor.writeSensorsToEEPROM();
  //powerSensor.readSensorsFromEEPROM();
  usleep(1000000);
  std::cout << "RUNNIT!!" << std::endl;
  powerSensor.mark();
  usleep(1000000);
}
