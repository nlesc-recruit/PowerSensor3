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


#include <avr/eeprom.h>
#include <Wire.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


#define MAX_SENSORS 5

struct EEPROM
{
  struct Sensor {
    double volt;
    double type;
    double nullLevel;
  } sensors[MAX_SENSORS];
} eeprom __attribute__((section(".eeprom")));

struct Sensor
{
  bool     inUse;
  uint16_t count;
  int32_t  total;
  double   weight;
  double   nullLevel;

  double power() const { return weight * total / count - nullLevel; }

} sensors[MAX_SENSORS];


bool streamValues = false;
uint8_t sendMarkerNext = 0;

inline uint8_t nextSensor(uint8_t currentSensor)
{
  do
    if (++ currentSensor == MAX_SENSORS)
      currentSensor = 0;
  while (!sensors[currentSensor].inUse);

  return currentSensor;
}


inline void setADMUX(uint8_t sensor)
{
#if defined __AVR_ATmega32U4__
  ADMUX = _BV(REFS0) | ((sensor <= 2 ? 6 : 4) - sensor);
#else
  ADMUX = _BV(REFS0) | (sensor + 1); // ADC0 reads the LCD buttons; skip it
#endif
}


ISR(ADC_vect)
{
  static uint8_t currentSensor = 0;

  uint8_t low = ADCL, high = ADCH; // read in this order

  // start next ADC ASAP.
  uint8_t previousSensor = currentSensor;
  currentSensor = nextSensor(currentSensor);

  setADMUX(currentSensor);
  ADCSRA |= _BV(ADSC); // start ADC conversion

  int16_t level = (high << 8) | low;
  sensors[previousSensor].total += level - 512;
  sensors[previousSensor].count ++;

  if (streamValues) {
    Serial.write(((previousSensor & 0x7) << 4) | (level & 0x3C0) | (1 << 7));
    Serial.write(((sendMarkerNext << 6) | (level & 0x3F)) & ~(1 << 7));
    
    sendMarkerNext = 0;
#if defined __AVR_ATmega32U4__
    Serial.flush();
#endif
  }
}


void configureFromEEPROM()
{
  EEPROM copy;
  eeprom_read_block(&copy, &eeprom, sizeof copy);

  for (unsigned i = 0; i < MAX_SENSORS; i ++) {
    sensors[i].inUse = copy.sensors[i].volt != 0;
    sensors[i].weight = sensors[i].inUse ? (2.5 / 512) * (copy.sensors[i].volt / copy.sensors[i].type) : 0;
    sensors[i].nullLevel = copy.sensors[i].nullLevel;
  }
}


void readConfig()
{
  EEPROM copy;
  eeprom_read_block(&copy, &eeprom, sizeof copy);
  Serial.write((const uint8_t *) &copy, sizeof copy);
}


void writeConfig()
{
#if !defined __AVR_ATmega32U4__
  Serial.begin(115200); // serial comm from host to Arduino seems less reliable --> reduce baud rate
#endif

  EEPROM copy;

  for (unsigned i = 0; i < sizeof copy; i ++) {
    while (Serial.available() == 0)
      ;

    ((uint8_t *) &copy)[i] = Serial.read();
  }

#if !defined __AVR_ATmega32U4__
  Serial.begin(2000000);
#endif

  eeprom_update_block(&copy, &eeprom, sizeof copy);
  configureFromEEPROM();
}


void serialEventRun()
{
  switch (Serial.read()) {
    case 'R': readConfig();
	      break;

    case 'W': writeConfig();
	      break;

    case 'S': streamValues = true;
	      break;

    case 'T': streamValues = false;
	      break;

    case 'X': streamValues = false;
	      Serial.write((const uint8_t []) { 0xFF, 0x3F }, 2);
	      break;

    case 'M': sendMarkerNext = 1;
              break;
  }
}


void setup()
{
  configureFromEEPROM();

  lcd.begin(16, 2);
  lcd.print("Sensor  :      W");
  lcd.setCursor(0, 1);
  lcd.print("Total:         W");

  Serial.begin(4000000);

  setADMUX(0);
  ADCSRA |= _BV(ADIE); // enable ADC interrupts
  ADCSRA |= _BV(ADSC); // start ADC conversion
}


void loop()
{
  static unsigned long previousPrintTime;
  static uint8_t count, currentSensor = nextSensor(MAX_SENSORS - 1), currentLine;

  uint16_t currentTime = millis(), timeDifference = currentTime - previousPrintTime;

  if (timeDifference > 333) {
    double power;

    if (currentLine == 0) {
      // print top line; rotate among the used sensors

      noInterrupts();
      power = sensors[currentSensor].power();
      interrupts();

      if ((++ count & 7) == 0)
	currentSensor = nextSensor(currentSensor);

      if ((count & 7) == 1) {
	lcd.setCursor(7, 0);
	lcd.print((char) ('0' + currentSensor));
      }
    } else { // currentLine == 1
      // print bottom line; the sum of all sensors
      power = 0;

      noInterrupts();

      for (uint8_t sensor = 0; sensor < MAX_SENSORS; sensor ++) {
	if (sensors[sensor].inUse) {
	  power += sensors[sensor].power();
	  sensors[sensor].total = 0;
	  sensors[sensor].count = 0;
	}
      }

      interrupts();

      previousPrintTime = currentTime;
    }

    char buffer[16];
    dtostrf(power, 6, 1, buffer);
    lcd.setCursor(9, currentLine);
    lcd.print(buffer);

    currentLine ^= 1;
  }
}
