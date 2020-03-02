#include <Arduino.h>

#include "eeprom_helper.h"

// defines;
#define MAX_SENSORS 3

// PowerSensor Serial variables
bool streamValues = false;
uint8_t sendMarkerNext = 0;

// Virtual adress table for the EEPROM emulation;
uint16_t VirtAddVarTab[MAX_SENSORS] = {0x5555, 0x6666, 0x7777};

struct Sensor {
  bool inUse;
  uint8_t pin;

  // EEPROM stored variables;
  float type;
  float volt;
  float nullLevel;

} sensors[MAX_SENSORS];

void configureSensors()
{
  // configure sensors from EEPROM?
  sensors[0].pin = PA4;
  sensors[0].inUse = true;
  sensors[0].nullLevel = 2.5;
  sensors[0].volt = 3.3;
  sensors[0].type = .185;
  sensors[1].pin = PA5;
  sensors[1].inUse = true;
  sensors[1].nullLevel = 2.5;
  sensors[1].volt = 3.3;
  sensors[1].type = .185;
  sensors[2].pin = PA6;
  sensors[2].inUse = true;
  sensors[2].nullLevel = 2.5;
  sensors[2].volt = 3.3;
  sensors[2].type = .185;
}

void writeSensorConfiguration()
{
  uint16_t halfWord[4];
  uint16_t virtAddress;
  uint32_t fullWord;

  for (int i = 0; i < MAX_SENSORS; i++)
  {
    memcpy(&fullWord, &sensors[i].nullLevel, 4);

    halfWord[0] = (uint16_t) (sensors[i].type * 1000);
    halfWord[1] = (uint16_t) (sensors[i].volt * 10);
    halfWord[2] = (fullWord >> 16) & 0xFFFF;
    halfWord[3] = fullWord & 0xFFFF;

    virtAddress = VirtAddVarTab[i];
    for (int j= 0; j < 4; j++) 
    {
      EE_WriteVariable(virtAddress, halfWord[j]);
      virtAddress++;
    }
  }

}

void readSensorConfiguration() 
{
  uint16_t halfWord[4];
  uint16_t virtAddress;
  uint32_t fullWord;

  for (int i = 0; i < MAX_SENSORS; i++)
  {
    virtAddress = VirtAddVarTab[i];
    for (int j= 0; j < 4; j++) 
    {
      EE_ReadVariable(virtAddress, &halfWord[j]);
      virtAddress++;
    }

    sensors[i].type = ((float) halfWord[0]) / 1000;
    sensors[i].volt = ((float) halfWord[1]) / 10;
    fullWord = (halfWord[2] << 16) | halfWord[3];
    memcpy(&sensors[i].nullLevel, &fullWord, 4);

    uint16_t level;

    if (sensors[i].nullLevel == 2.5 && sensors[i].volt == 3.30) //&& sensors[i].type == .185 )
    {
      level = 254;
    } else
    {
      level = 253;
    }
    
    Serial.write(((i & 0x7) << 4) | ((level & 0x3C0) >> 6) | (1 << 7));
    Serial.write(((sendMarkerNext << 6) | (level & 0x3F)) & ~(1 << 7)); 
  }
}

uint8_t nextSensor(uint8_t currentSensor) 
{
  do
  {
    if (++ currentSensor == MAX_SENSORS)
      currentSensor = 0;
  } while (!sensors[currentSensor].inUse);

  return currentSensor;
}

// is called once per loop();
void serialEvent() 
{
  // checks if there is something in the input buffer, ensures the function read() does not block;
  if (Serial.available() > 0) 
  {
    switch (Serial.read())
    {
      case 'R':
        readSensorConfiguration();
        break;

      case 'W':
        writeSensorConfiguration();
        break;

      // S: start character, turns the streaming of values on;
      case 'S':
        //streamValues = true;
        break;

      // T: stop character, turns the streaming of values off;
      case 'T':
        //streamValues = false;
        break;

      // X: shutdown character, turns the stream off and kills the IOthread;
      case 'X':
        //streamValues = false;
        Serial.write((const uint8_t []) { 0xFF, 0xE0}, 2);
        break;
  
      case 'M': // M: marker character, places a marker in the output file;
        sendMarkerNext = 1;
        break;

      default:
        break;
    }
  }
}


// "__irq_adc" is used as weak method in stm32f407 CMSIS startup file;
void ADC_Handler(void)
{
  // read out the Data Register of ADC1;
  uint16_t level = ADC1_BASE->DR;

  static uint8_t currentSensor = 0;

  
  if (streamValues) 
  {
    // write the level, write() only writes per byte;
    Serial.write(((currentSensor & 0x7) << 4) | ((level & 0x3C0) >> 6) | (1 << 7));// 0x80 | (currentSensor << 4) | (level >> 6));
    Serial.write(((sendMarkerNext << 6) | (level & 0x3F)) & ~(1 << 7)); //(sendMarkerNext << 6) | (level & 0x3F));
    // reset the marker
    sendMarkerNext = 0;
  }

  // get next sensor in line to convert from;
  currentSensor = nextSensor(currentSensor);

  // set pin in sequence register 3 to be input for conversion;
  ADC1_BASE->SQR3 &= ~(1 << 0);
  ADC1_BASE->SQR3 &= ~(1 << 1);
  ADC1_BASE->SQR3 &= ~(1 << 2);
  ADC1_BASE->SQR3 &= ~(1 << 3);
  ADC1_BASE->SQR3 |= sensors[currentSensor].pin; // |= PA5; |= PA6;

  // set Start conversion bit in Control Register 2;
  ADC1_BASE->CR2 |= ADC_CR2_SWSTART;
}

void setup() 
{
  // baudrate 4M for development, runs at max 1M baud, uses SerialUSB (not tested);
  Serial.begin(4000000); 

  // Unlock the flash for EEPROM emulation;
  FLASH_Unlock();

  // Initialize EEPROM, this will also check for valid pages;
  if (EE_Init() != FLASH_COMPLETE)
  {
    Serial.write((const uint8_t []) { 0xFF, 0xE0}, 2);
  }

  // set ADON bit in ADC control register to turn the converter on;
  ADC1_BASE->CR2 |= ADC_CR2_ADON; 

  // set PA4 in sequence register 3 to be the first input for conversion;
  ADC1_BASE->SQR3 |= PA4; // |= PA5; |= PA6;

  // set PA4, PA5, PA6 to analog input in the GPIO mode register;
  GPIOA_BASE->MODER |= 0x00003F00;

  // set End of Conversion Interrupt Enable bit to enable interrupts;
  ADC1_BASE->CR1 |= ADC_CR1_EOCIE;

  // set Resolution bits to 10 bit resolution;
  ADC1_BASE->CR1 |= 0x01000000;


  // ##########################################################
  // #            Interrupt things that do not work           #
  // ##########################################################
  // SCB_BASE->VTOR = 0x08000000 | (0 & 0x1FFFFF80);
  // NVIC_BASE->ISER[0] |= BIT(NVIC_ADC_1_2 % 32);
  // nvic_init(0x08000000, 0);
  // nvic_irq_enable(NVIC_ADC_1_2);
  // nvic_irq_set_priority(NVIC_ADC_1_2, 3);
  // ##########################################################


  // set Start conversion bit in Control Register 2;
  ADC1_BASE->CR2 |= ADC_CR2_SWSTART;

  // configure sensors;
  configureSensors();
}

void loop() 
{
  //delay(2000);

  //Serial.println((uint16_t) (3.3 * 10));
  //Serial.println(((float) 33) / 10);
  // check if the End of Conversion bit is set in the ADC1 Status Register;
  if (ADC1_BASE->SR & ADC_SR_EOC) 
  {
    ADC_Handler();
  }
  
  // manually check if there is input from the host;
  serialEvent();
}

