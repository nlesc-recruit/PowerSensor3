// Includes
#include <Arduino.h>

// Defines
#define MAX_SENSORS 3
#define __PRINT__
struct Sensor
{
  bool     inUse;
  uint16_t count;
  int32_t  total;
  double   weight;
  double   nullLevel;

  double   currentLevel; //remove this later

  double power() const { return weight * total / count - nullLevel; }

} sensors[MAX_SENSORS];

bool streamValues = false;

inline void setADMUX(uint8_t sensor)
{ // The REFS0 bit is used to determine what reference voltage source to be used for AD Conversion. Other words: which channel or pin.
  #if defined __AVR_ATmega32U4__
  ADMUX = _BV(REFS0) | ((sensor <= 2 ? 6 : 4) - sensor);
  #else
  ADMUX = _BV(REFS0) | (sensor + 1); // ADC0 reads the LCD buttons; skip it
  #endif
}

inline uint8_t nextSensor(uint8_t currentSensor)
{
  if (++ currentSensor == MAX_SENSORS)
    currentSensor = 0;

  return currentSensor;
}

ISR(ADC_vect)
{ // Interrupt service routine
  static uint8_t currentSensor = 0;

  // Since the Digital value of corresponding Analog vary from 0 to 1024, value can't be stored in a single register
  // that's why two registers (ADCH & ADCL) are used to store that digital value.
  uint8_t low = ADCL, high = ADCH; // read in this order. They are right-orientated

  // start next ADC ASAP.
  uint8_t previousSensor = currentSensor;
  currentSensor = nextSensor(currentSensor);

  setADMUX(currentSensor); // set the ADC multiplexer to the channel we want to read next.
  ADCSRA |= _BV(ADSC); // start ADC conversion

  int16_t level = (high << 8) | low;
  sensors[currentSensor].currentLevel = (((level/512.0)*2.5)); //<-- used to get mVolts

  #if defined __PRINT__
  String R = "\r";
  for (unsigned i = 0; i < MAX_SENSORS; i++) {
    if (sensors[i].currentLevel >= 2.5) {
      R += "HIGH";
    } else {
      R += "LOW ";
    }
    R += "  ";
  }
  Serial.print(R);
  #endif

  sensors[previousSensor].total += level - 512;
  sensors[previousSensor].count ++;

  if (streamValues)
  {
    Serial.write((previousSensor << 5) | (level >> 5)); // Writes first 3 bits for sensorID and last 5 bits as upper parts of level bits.
    Serial.write(0xE0 | (level & 0x1F));
  }
}

float retrieveNullLevel(uint8_t currentSensor)
{
  uint8_t low = ADCL, high = ADCH;
  setADMUX(currentSensor);
  ADCSRA |= _BV(ADSC); // start ADC conversion
  return ((((high << 8) | low)/512.0)*2.5);
}

void configureSensors(double volt)
{
  for (unsigned i = 0; i < MAX_SENSORS; i ++) {
    sensors[i].inUse = true; //Sensors now always in use else: copy.sensors[i].volt != 0;
    sensors[i].weight = sensors[i].inUse ? (2.5 / 512) * (volt / .185) : 0; //copy.sensors[i].type is ACS712-05 for now.
    sensors[i].nullLevel = 2.5; //mVolt; copy.sensors[i].nullLevel if it is calibrated else take 1 measurement

    sensors[i].currentLevel = 0; //remove this later
  }
}

void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  //ADCSRA |= _BV(ADIE); // Analog-to-Digital Interrupt Enable
  //ADCSRA |= _BV(ADSC); // Analog-to-Digital Start Conversion
  configureSensors(3.3); // now configured at 3.3 volts

  #if defined __PRINT__
  Serial.println(" ");
  Serial.println("SEN1  SEN2  SEN3");
  #endif

  setADMUX(0); // Start measuring with the first sensor
  ADCSRA |= _BV(ADIE); // Analog-to-Digital Interrupt Enable
  ADCSRA |= _BV(ADSC); // Analog-to-Digital Start Conversion
}

void loop()
{
  // do nothing
}
