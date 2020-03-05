#include <Arduino.h>

#include "eeprom_helper.h"
#include "dma.h"

// defines;
#define MAX_SENSORS 3

// PowerSensor Serial variables
bool streamValues = false;
uint8_t sendMarkerNext = 0;

// Virtual adress table for the EEPROM emulation;
uint16_t VirtAddVarTab[MAX_SENSORS] = {0x5555, 0x6666, 0x7777};

uint16_t dmaBuffer[MAX_SENSORS];

struct EEPROM{
  struct Sensor
  {
    float type;
    float volt;
    float nullLevel;
  } sensors[MAX_SENSORS];
};

struct Sensor {
  bool inUse;
  uint8_t pin;
  float type;
  float volt;
  float nullLevel;

} sensors[MAX_SENSORS];

bool approximates(float a, float b)
{
  return a / b > .999999 && a / b < 1.000001;
}

bool conversionComplete()
{
  if (ADC1_BASE->SR & ADC_SR_EOC)
  {
    return true;
  }
  else
  {
    return false;
  }
}

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

void writeConfigurationToEEPROM(EEPROM recv)
{
  uint16_t halfWord[4];
  uint32_t fullWord;

  uint16_t virtualBaseAddress = 0x1111;
  uint16_t virtualVariableAddress;

  for (int i = 0; i < MAX_SENSORS; i++)
  {
    virtualVariableAddress = virtualBaseAddress;
    memcpy(&fullWord, &recv.sensors[i].nullLevel, 4);
    halfWord[0] = (uint16_t) (recv.sensors[i].type * 1000);
    halfWord[1] = (uint16_t) (recv.sensors[i].volt * 1000);
    halfWord[2] = (fullWord >> 16) & 0xFFFF;
    halfWord[3] = fullWord & 0xFFFF;

    for (int j= 0; j < 4; j++)
    {
      EE_WriteVariable(virtualVariableAddress, halfWord[j]);
      virtualVariableAddress++;
    }
    virtualBaseAddress += 0x1111;
  }
}

EEPROM readSensorConfiguration()
{
  EEPROM copy;

  uint16_t halfWord[4];
  uint32_t fullWord;

  uint16_t virtualBaseAddress = 0x1111;
  uint16_t virtualVariableAddress;

  for (int i = 0; i < MAX_SENSORS; i++)
  {
    virtualVariableAddress = virtualBaseAddress;
    for (int j = 0; j < 4; j++)
    {
      EE_ReadVariable(virtualVariableAddress, &halfWord[j]);
      virtualVariableAddress++;
    }
    copy.sensors[i].type = ((float) halfWord[0]) / 1000;
    copy.sensors[i].volt = ((float) halfWord[1]) / 1000;
    fullWord = (halfWord[2] << 16) | halfWord[3];
    memcpy(&copy.sensors[i].nullLevel, &fullWord, 4);
    virtualBaseAddress += 0x1111;
  }

  return copy;
}

void configureFromEEEPROM()
{
  EEPROM copy = readSensorConfiguration();
  for (int i = 0; i < MAX_SENSORS; i++)
  {
    sensors[i].type = copy.sensors[i].type;
    sensors[i].volt = copy.sensors[i].type;
    sensors[i].nullLevel = copy.sensors[i].nullLevel;
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
  for (int i = 0; i < MAX_SENSORS; i++)
  {
    // put the corresponding value from the buffer in level;
    uint16_t level = dmaBuffer[i];

    // init current sensor;
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

  }
  // clear SQR conversion channel register;
  //ADC1_BASE->SQR3 &= ~(0xFFF);

  // set pin in sequence register 3 to be input for conversion;
  //ADC1_BASE->SQR3 |= sensors[currentSensor].pin; // |= PA5; |= PA6;

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
    // if eeprom failes to initialize send kill signal?? (bit useless);
    Serial.write((const uint8_t []) { 0xFF, 0xE0}, 2);
  }

  // set ADON bit in ADC control register to turn the converter on;
  ADC1_BASE->CR2 |= ADC_CR2_ADON;

  // enable ADC system clock;
  RCC_BASE->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // enable DMA system clock;
  RCC_BASE->AHB1ENR |= RCC_AHBENR_DMA1EN;

  // set PA4 in sequence register 3 to be the first input for conversion;
  ADC1_BASE->SQR3 |= PA4 | (PA5 << 5) | (PA6 << 10); // |= PA5; |= PA6;

  // set amount of conversions to 3 (0 = 1 conversion);
  ADC1_BASE->SQR1 |= (2 << ADC_SQR1_L);

  // set PA4, PA5, PA6 to analog input in the GPIO mode register;
  GPIOA_BASE->MODER |= 0x00003F00;

  // set End of Conversion Interrupt Enable bit to enable interrupts;
  ADC1_BASE->CR1 |= ADC_CR1_EOCIE;

  // set Resolution bits to 10 bit resolution;
  ADC1_BASE->CR1 |= 0x01000000;

  // enable DMA in ADC control register 2;
  ADC1_BASE->CR2 |= ADC_CR2_DMA;

  // set 9th bit in ADC control register 2 ??;
  ADC1_BASE->CR2 |= (0x1 << 9);

  /* If the stream is enabled, disable it by resetting the EN bit in the DMA_SxCR register,
     then read this bit in order to confirm that there is no ongoing stream operation. */
  DMA2_BASE->STREAM[0].CR &= ~(DMA_CR_EN);

  while (DMA2_BASE->STREAM[0].CR & DMA_CR_EN)
  {

  }

  /* Set the peripheral port register address in the DMA_SxPAR register. The data will be
    moved from/ to this address to/ from the peripheral port after the peripheral event. */
  DMA2_BASE->STREAM[0].PAR |= (uint32_t) &ADC1_BASE->DR;

  /* Set the memory address in the DMA_SxMA0R register. The data will be written to or read
    from this memory after the peripheral event. */
  DMA2_BASE->STREAM[0].M0AR |= (uint32_t) &dmaBuffer;

  /* Configure the total number of data items to be transferred in the DMA_SxNDTR
    register. After each peripheral event or each beat of the burst, this value is
    decremented. */
  DMA2_BASE->STREAM[0].NDTR |= MAX_SENSORS;

  /* Select the DMA channel (request) using CHSEL[2:0] in the DMA_SxCR register. */



  // set memory unit size to 16 bits;
  DMA2_BASE->STREAM[0].CR |= DMA_CR_MSIZE_16BITS;

  // set memory target address to the created DMA buffer;


  // set pheripheral unit size to 16 bits;
  DMA2_BASE->STREAM[0].CR |= DMA_CR_PSIZE_16BITS;

  // set pheripheral target address to the data register of ADC1;


  // set number of conversions in one sequence to 3, for 3 sensors;


  // set memory increment on, so that it will fill the buffer/array correctly;
  DMA2_BASE->STREAM[0].CR |= DMA_CR_MINC;

  // set DMA to circular mode so it will refill the increment once its empty;
  DMA2_BASE->STREAM[0].CR |= DMA_CIRC_MODE;

  // set priority level of the DMA stream to very high;
  DMA2_BASE->STREAM[0].CR |= DMA_CR_PL_VERY_HIGH;

  // after all configurations are done, set the enable bit (from this on the registers are read-only);
  DMA2_BASE->STREAM[0].CR |= DMA_CR_EN;


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

  // set ADC1 to continuous conversion mode;
  ADC1_BASE->CR2 |= ADC_CR2_CONT;

  // configure sensors;
  configureSensors();
}

void loop()
{
  // check if the conversion has ended;
  if (conversionComplete())
  {
    // run the ADC handler script;
    ADC_Handler();
  }

  // manually check if there is input from the host;
  serialEvent();
}

