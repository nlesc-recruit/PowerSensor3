#include "PowerSensor.h"

namespace PowerSensor {

  PowerSensor::PowerSensor(const char *device)
  :
    startTime(omp_get_wtime()),
    fd(openDevice(device)),
    thread(nullptr)
  {
    startCleanupProcess();
    readSensorsFromEEPROM();
    startIOthread();
  }


  PowerSensor::~PowerSensor()
  {
    stopIOthread();

    if (close(fd))
      perror("close device");
  }
}
