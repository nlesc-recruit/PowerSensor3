#ifndef __POWER_SENSOR_H
#define __POWER_SENSOR_H

namespace PowerSensor {

  class PowerSensor {
    public:
      PowerSensor(const char* device);
      ~PowerSensor();

    private:
      int fd;
      int openDevice(const char* device);
  };


}  // namespace PowerSensor

#endif  // __POWER_SENSOR_H