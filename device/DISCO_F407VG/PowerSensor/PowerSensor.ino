#include <Arduino.h>
#include "eeprom_emulation.h"

// define max sensors and emulated EEPROM base address
#define MAX_SENSORS 	5
#define BASE_ADDRESS	0x1111

// Bool to define whether to stream values;
bool streamValues = false;

// Int which is send to host, if 1 then it is a marked value;
uint8_t sendMarkerNext = 0;

// buffer for the DMA to transfer level values to;
uint16_t dmaBuffer[MAX_SENSORS];

// keeps the amount of sensors for ADC and DMA configs;
uint8_t activeSensorCount = 0;

// EEPROM struct to save in emulated EEPROM
struct EEPROM {
  struct Sensor
  {
    float volt;
    float type;
    float nullLevel;
  } sensors[MAX_SENSORS];
};

// Calculates the size of the EEPROM struct in halfwords (16b);
const uint16_t eepromSize = sizeof(EEPROM) / 2;

// Reserves appropriate amount of memory for virtual address table;
uint16_t VirtAddVarTab[eepromSize];

struct Sensor
{
  bool inUse;
} sensors[MAX_SENSORS];

// writes configuration recv to EEPROM
void writeConfigurationToEEPROM(EEPROM recv)
{
  // set a pointer of 16 bits to the start of the struct;
  uint16_t *p_recv = (uint16_t *)&recv.sensors[0];

  // write the data to the EEPROM;
  for (int i = 0; i < eepromSize; i++)
  {
    EE_WriteVariable(VirtAddVarTab[i], *p_recv);
    p_recv++; 
    delay(1); 
  }
}

// returns configuration from EEPROM;
EEPROM readSensorConfiguration()
{
  // create struct to put variables in;
  EEPROM copy;

  // set a pointer to the start of the struct; 
  uint16_t *p_copy = (uint16_t *)&copy.sensors[0];

  // read the data from the EEPROM;
  for (int i = 0; i < eepromSize; i++)
  {
    EE_ReadVariable(VirtAddVarTab[i], p_copy);
    p_copy++;
    delay(1);
  }

  return copy;
}

void configureFromEEEPROM()
{
  EEPROM copy = readSensorConfiguration();
  for (int i = 0; i < MAX_SENSORS; i++)
  {
    sensors[i].inUse = false;
    if (copy.sensors[i].volt != 0)
    {
      sensors[i].inUse = true;
    }
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

void readConfig()
{
  EEPROM send = readSensorConfiguration();
  Serial.write((const uint8_t *) &send, sizeof send);
}

void writeConfig()
{
  // create EEPROM instance;
  EEPROM recv;
  for (int i = 0; i < sizeof recv; i++)
  {
    while (Serial.available() == 0)
      ;
    ((uint8_t *) &recv)[i] = Serial.read();
  }
  writeConfigurationToEEPROM(recv);
  configureFromEEEPROM();
  activeSensorCount = 0;
  
  // reconfigure ADC and DMA;
  configureADC();
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
        readConfig();
        break;

      case 'W':
        writeConfig();
        break;

      // S: start character, turns the streaming of values on;
      case 'S':
        streamValues = true;
        break;

      // T: stop character, turns the streaming of values off;
      case 'T':
        streamValues = false;
        break;

      // X: shutdown character, turns the stream off and kills the IOthread;
      case 'X':
        streamValues = false;
	Serial.write((const uint8_t []) { 0xFF, 0x3F}, 2);
        Serial.write((const uint8_t []) { 0xFF, 0x3F}, 2);
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
  if (streamValues)
  {  
  // for every sensor there is send value;
  for (int i = 0; i < activeSensorCount; i++)
  {
    uint16_t level = dmaBuffer[i]; //ADC1_BASE->DR;

    // write the level, write() only writes per byte;
    Serial.write(((i & 0x7) << 4) | ((level & 0x3C0) >> 6) | (1 << 7));
    Serial.write(((sendMarkerNext << 6) | (level & 0x3F)) & ~(1 << 7));
      
    // reset the marker
    sendMarkerNext = 0;
    }
  }
}

void configureDMA()
{ 
  // disable the stream, the dma registers cannot be altered when this bit is 1;
  DMA2_Stream0->CR &= ~((uint32_t)DMA_SxCR_EN);

  // wait for the EN bit to go to 0;
  while (DMA2_Stream0->CR & DMA_SxCR_EN)
  {
    // do nothing (maybe add timeout later);
  }

  // set the peripheral address to ADC1's data register;
  DMA2_Stream0->PAR |= (uint32_t) &ADC1->DR;

  // set memory unit size to 16 bits, ADC resolution is 10;
  DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0;

  // set memory target address to the created DMA buffer;
  DMA2_Stream0->M0AR |= (uint32_t) &dmaBuffer;

  // set pheripheral unit size to 16 bits;
  DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0;

  // set pheripheral target address to the data register of ADC1;
  DMA2_Stream0->PAR |= (uint32_t) &ADC1->DR;

  // set number of conversions in one sequence to the amount of active sensors;
  DMA2_Stream0->NDTR &= ~(0xF);
  DMA2_Stream0->NDTR |= activeSensorCount;

  // set memory increment on, so that it will fill the buffer/array correctly;
  DMA2_Stream0->CR |= DMA_SxCR_MINC;

  // set DMA to circular mode so it will refill the increment once its empty;
  DMA2_Stream0->CR |= DMA_SxCR_CIRC;

  // after all configurations are done, set the enable bit;
  DMA2_Stream0->CR |= DMA_SxCR_EN;
}

void configureADC()
{  
  // ensure the ADC is off;
  ADC1->CR2 &= ~(ADC_CR2_SWSTART);
  ADC1->CR2 &= ~(ADC_CR2_ADON);

  // initialise sensors;
  for (int i = 0; i < MAX_SENSORS; i++)
  { 
    // clears input channel and pin for sensor;
    ADC1->SQR3 &= ~(0x1F << (i * 5));
    GPIOA->MODER &= ~(0x3 << (i * 2));
    
    // set a sensor input channel and pin if it is in use;
    if (sensors[i].inUse)
    {
      // set sensor input channel in sequence registers;
      ADC1->SQR3 |= (i << (activeSensorCount * 5));

      // set pins to analog input in the GPIO mode register;
      GPIOA->MODER |= (0x3 << (activeSensorCount * 2));
      
      // keep track of active sensor count for the DMA;
      activeSensorCount++;
    }
  }

  // set amount of conversions to 3 (0 = 1 conversion);
  ADC1->SQR1 &= ~((0xF) << 20);
  ADC1->SQR1 |= ((activeSensorCount - 1) << 20);

  // set Resolution bits to 10 bit resolution;
  ADC1->CR1 |= 0x01000000;

  // enable scan mode to scan for next channel for conversion
  ADC1->CR1 |= ADC_CR1_SCAN;

  // DMA requests are issued as long as data are converted and DMA=1;
  ADC1->CR2 |= (1 << 9); // DDS bit
  
  // run the configuration of the DMA;
  configureDMA();

  // enable the DMA in the ADC after the DMA is configured;
  ADC1->CR2 |= ADC_CR2_DMA;

  // set ADON bit in ADC control register to turn the converter on;
  ADC1->CR2 |= ADC_CR2_ADON; 

  // enable continuous mode
  ADC1->CR2 |= ADC_CR2_CONT;

  // set ADON bit in ADC control register for the second time to actually turn the converter on;
  ADC1->CR2 |= ADC_CR2_ADON; 

  // start the conversions
  ADC1->CR2 |= ADC_CR2_SWSTART;
}

void generateVirtualAddresses()
{
  uint16_t addr = BASE_ADDRESS;
  for (int i = 0; i < eepromSize; i++)
  {
    VirtAddVarTab[i] = addr;
    addr++;
  }
}

void setup()
{
  // baudrate 4M;
  Serial.begin(40000000);

  // populate VirtAddVarTab memory with addresses incremented from BASE;
  generateVirtualAddresses();

  // unlock flash memory;
  HAL_FLASH_Unlock();

  // initialize emulated EEPROM, check if there are active pages;
  EE_Init();
  
  // configure the sensors from emulated EEPROM;
  configureFromEEEPROM();

  // enable ADC system clock;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // enable DMA system clock;
  RCC->AHB1ENR |= 0x400000;

  // configure the ADC;
  configureADC();
}

void loop()
{ 
  // check if the conversion has ended;
  if (DMA2->LISR & (1 << 4))
  {
    ADC_Handler();
  }
  // manually check if there is input from the host;
  serialEvent();  
}
