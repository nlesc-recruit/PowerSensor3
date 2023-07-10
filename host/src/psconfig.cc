#include <unistd.h>

#include <cstring>
#include <iostream>
#include <memory>
#if __GNUC__ < 8
#include <experimental/filesystem>
#define fs std::experimental::filesystem
#else
#include <filesystem>
#define fs std::filesystem
#endif
#include <chrono>
#include <sstream>

#include "PowerSensor.hpp"


std::unique_ptr<PowerSensor3::PowerSensor> powerSensor;
unsigned int sensor = 0;


unsigned int selectSensor(unsigned int sensor) {
  if (sensor >= PowerSensor3::MAX_SENSORS) {
    std::cerr << "Invalid sensor ID: " << sensor << ", max value is " << PowerSensor3::MAX_SENSORS - 1 << std::endl;
  }
  return sensor;
}


PowerSensor3::PowerSensor *getPowerSensor(std::string device) {
  if (device.empty())
    device = "/dev/ttyACM0";
  if (powerSensor.get() == nullptr)
    powerSensor = std::unique_ptr<PowerSensor3::PowerSensor>(new PowerSensor3::PowerSensor(device));

  return powerSensor.get();
}

float getDefaultSensitivity(std::string type) {
  // Sensitivity is given in mV/A for current sensors, or mV/V for voltage sensors
  float sensitivity = 0;

  // even sensors are current sensor, odd sensors are voltage sensors
  if ((sensor % 2) == 0) {
    // Current sensors. These are of type MLX91221KDF-ABF-0NN-RE, where NN is the number after
    // MLX in the shortened name
    if (type.compare("MLX10") == 0) {
      sensitivity = 120.;
    } else if (type.compare("MLX20") == 0) {
      sensitivity = 62.5;
    } else if (type.compare("MLX50") == 0) {
      sensitivity = 25.0;
    } else if (type.compare("MLX75") == 0) {
      sensitivity = 16.67;
    } else {
      std::cerr << "No sensitivity known for current sensor of type " << type << "."
                  " Please make sure to set sensitivity manually with the -n option." << std::endl;
    }
    // convert from mV/A to V/A
    sensitivity /= 1000.;
  } else {
    // voltage sensor. These should have a gain of ~unity.
    // In the future could add different types here corresponding to different voltage dividers
    sensitivity = 1.;
  }
  return sensitivity;
}


void measureSensors(PowerSensor3::State* startState, PowerSensor3::State* stopState) {
  *startState = powerSensor->read();
  sleep(2);
  *stopState = powerSensor->read();
}


void autoCalibrate() {
  // dump data to file for a second
  fs::path dumpFile = fs::temp_directory_path() / fs::path("PowerSensorTmpFile.txt");
  powerSensor->dump(dumpFile.string());
  std::this_thread::sleep_for(std::chrono::seconds(1));
  powerSensor->dump("");

  // read data from file
  std::ifstream inFile(dumpFile);
  std::string line;
  char marker;
  float value;
  double sum = 0;
  int nvalues = 0;
  while (std::getline(inFile, line)) {
    std::stringstream ss(line);
    ss >> marker;
    // only if the marker is S, this is valid line with sensor values
    if (marker != 'S')
      continue;
    // read the three time values (these will be ignored)
    ss >> value >> value >> value;
    // now there are 3 values per sensor (current, voltage, power)
    // ignore until the correct sensor pair is reached. pair ID = sensor ID / 2
    // next value is current, ignore one more value if voltage needs to be calibrated (odd sensor ID)
    for (unsigned int s = 0; s < 3 * (sensor / 2) + (sensor % 2); s++) {
      ss >> value;
    }
    // finally get the sensor value and store it
    ss >> value;
    sum += value;
    nvalues++;
  }
  // delete the dump file
  if (std::remove(dumpFile.c_str()) != 0) {
    perror("remove tmpfile");
  }
  // get the average value
  value = sum / nvalues;
  // assume the output value should have been zero
  // calculate new vref based on earlier calibration values
  float sensitivity = powerSensor->getSensitivity(sensor);
  float vref = powerSensor->getVref(sensor);
  float vref_new = sensitivity * value + vref;
  std::cout << "Result of autoCal for sensor " << sensor << ":"
  " old Vref: " << vref << " new Vref: " << vref_new << std::endl;
  // write new vref
  powerSensor->setVref(sensor, vref_new);
}


void print() {
  PowerSensor3::State startState, stopState;

  measureSensors(&startState, &stopState);

  std::string sensorType, unit, sensitivityName;
  int factor;
  for (unsigned sensor = 0; sensor < PowerSensor3::MAX_SENSORS; sensor++) {
    if (sensor % 2 == 0) {
      sensorType = "current";
      unit = " mV/A";
      sensitivityName = "Sensitivity";
      factor = 1000;  // to convert from V/A to mV/A
    } else {
      sensorType = "voltage";
      unit = "";
      sensitivityName = "Gain";
      factor = 1;
    }

    std::cout << "sensor " << sensor << " (" << sensorType << "): "
      "type: " << powerSensor->getType(sensor) << ", "
      "Vref: " << powerSensor->getVref(sensor) << " V, " <<
      sensitivityName << ": " << factor * powerSensor->getSensitivity(sensor) << unit << ", "
      "Status: " << (powerSensor->getInUse(sensor) ? "on" : "off") << std::endl;
  }

  double totalUsage = 0;
  for (unsigned int pair = 0; pair < PowerSensor3::MAX_PAIRS; pair++) {
      double usage = Watt(startState, stopState, pair);
      totalUsage += usage;
      std::cout << "Current usage pair " << pair << ": " << usage << " W" << std::endl;
  }
  std::cout << "Total usage: " << totalUsage << " W" << std::endl;
}


void usage(char *argv[]) {
  std::cerr << "usage: " << argv[0] << " [-h] [-d device] [-s sensor] [-t type] "
    "[-a | -v volt] [-n sensitivity] [-o on/off] [-p] [-l]" << std::endl;
  std::cerr << "-h prints this help" << std::endl;
  std::cerr << "-d selects the device (default: /dev/ttyACM0)" << std::endl;
  std::cerr << "-s selects the sensor (0-" << PowerSensor3::MAX_SENSORS << ")" << std::endl;
  std::cerr << "-t sets the sensor type. This also sets the sensitivity to the default value if "
               "the sensor is of a type known to this programme (see list at the bottom of this help)." << std::endl;
  std::cerr << "-v sets the reference voltage level" << std::endl;
  std::cerr << "-a automatically calibrate vref of the current sensor. "
               "The input to the sensor must be zero volt or ampere" << std::endl;
  std::cerr << "-n set the sensitivity in mV/A for current sensors (even sensors) "
               "or unitless gain for voltage sensors (odd sensors)" << std::endl;
  std::cerr << "-o turns a sensor on (1) or off (0)" << std::endl;
  std::cerr << "-p prints configured values" << std::endl;
  std::cerr << "-l toggles device display on/off" << std::endl;
  std::cerr << "example: " << argv[0] << " -d /dev/ttyACM0 -s 0 -t MLX10 -v 1.65 "
               "-o 1 -s 1 -t voltage0 -v 0 -n 0.95 -o 1 -p" << std::endl;
  std::cerr << "Known current sensor types: MLX10, MLX20, MLX50, MLX75." << std::endl;
  exit(1);
}


int main(int argc, char *argv[]) {
  std::string device;
  bool doWriteConfig = false;
  bool doPrint = false;

  std::cout << "psconfig version " <<  PowerSensor3::POWERSENSOR_VERSION << std::endl << std::endl;
  for (int opt; (opt = getopt(argc, argv, "d:s:i:t:av:n:o:lph")) >= 0;) {
    switch (opt) {
      // device select
      case 'd':
        device = optarg;
        break;

      // sensor select
      case 's':
        sensor = selectSensor((unsigned) atoi(optarg));
        break;

      // sensor type, also sets default sensitivity
      case 't': {
        getPowerSensor(device)->setType(sensor, optarg);
        // set default sensitivity (zero for unknown sensor)
        float sensitivity = getDefaultSensitivity(optarg);
        if (sensitivity > 0)
          getPowerSensor(device)->setSensitivity(sensor, sensitivity);
        doWriteConfig = true;
        break;
      }

      // sensor auto calibration of reference voltage
      case 'a':
        getPowerSensor(device);
        autoCalibrate();
        doWriteConfig = true;
        break;

      // sensor reference voltage
      case 'v':
        getPowerSensor(device)->setVref(sensor, atof(optarg));
        doWriteConfig = true;
        break;

      // sensor sensitivity
      case 'n':
        // sensitivity is given in mV/A for current sensors (even sensors) and should be scaled by a factor 1000
        // this is not necessary for voltage sensors (odd sensors)
        getPowerSensor(device)->setSensitivity(sensor, (sensor % 2) == 0 ? atof(optarg) / 1000: atof(optarg));
        doWriteConfig = true;
        break;

      // sensor on/off
      case 'o':
        getPowerSensor(device)->setInUse(sensor, static_cast<bool>(atoi(optarg)));
        doWriteConfig = true;
        break;

      // toggle display
      case 'l': {
        bool displayStatus = getPowerSensor(device)->toggleDisplay();
        std::cout << "Display is now " << (displayStatus ? "on" : "off") << "." << std::endl;
        break;
      }

      // print
      case 'p':
        doPrint = true;
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

  if (doWriteConfig) {
    getPowerSensor(device)->writeSensorsToEEPROM();
  }

  if (doPrint) {
    getPowerSensor(device);
    print();
  }

  return 0;
}
