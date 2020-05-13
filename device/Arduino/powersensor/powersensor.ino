// #include <stm32_eeprom.h>
//#include <stm32f4xx.h>

#include <Arduino.h>
#include "eeprom_emulation.h"

// defines;
#define MAX_SENSORS 3

// PowerSensor Serial variables
bool streamValues = false;
uint8_t sendMarkerNext = 0;

// Virtual adress table for the EEPROM emulation;
uint16_t VirtAddVarTab[MAX_SENSORS] = {0x5555, 0x6666, 0x7777};

// buffer for the DMA to transfer level values to;
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
  // check if the DMA is done with its sequence;
  if (DMA2->LISR & (1 << 4))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void writeConfigurationToEEPROM(EEPROM recv)
{
  // get the size of the received struct; should become constant
  uint16_t recvSize = sizeof(recv) / 2;

  // the virtualvariables always start at adress 0x1111 (this is arbitrary);
  uint16_t virtualVariableAddress = 0x1111;

  // set a pointer of 16 bits to the start of the struct;
  uint16_t *p_recv = (uint16_t *)&recv.sensors[0];

  // write the data to the EEPROM;
  for (int i = 0; i < recvSize; i++)
  {
    // uint16_t halfWord = 0;
    // memcpy(&halfWord, p_recv, sizeof(uint16_t));  // TODO: this memcpy can go, just use 16 bits pointer in write()
    EE_WriteVariable(virtualVariableAddress, *p_recv);
    p_recv++; 
    virtualVariableAddress++;
    delay(1); 
  }
}

EEPROM readSensorConfiguration()
{
  // create struct to put variables in;
  EEPROM copy;

  uint16_t copySize = sizeof(copy) /2; // should become constant

  // the virtual variables always start at address 0x1111;
  uint16_t virtualVariableAddress = 0x1111;

  // set a pointer to the start of the struct; 
  uint16_t *p_copy = (uint16_t *)&copy.sensors[0];

  // read the data from the EEPROM;
  for (int i = 0; i < copySize; i++)
  {
    //uint16_t halfWord;
    EE_ReadVariable(virtualVariableAddress, p_copy);
    //memcpy(p_copy, &halfWord, sizeof(uint16_t));
    p_copy++;
    virtualVariableAddress++;
    delay(1);
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
  // put the corresponding value from the buffer in level;
  //static uint8_t currentSensor = 0;
  //Serial.println("ad");  
  for (int i = 0; i < MAX_SENSORS; i++)
  {
    uint16_t level = dmaBuffer[i]; //ADC1_BASE->DR;

    if (streamValues) 
    {
      // write the level, write() only writes per byte;
      Serial.write(((i & 0x7) << 4) | ((level & 0x3C0) >> 6) | (1 << 7));
      Serial.write(((sendMarkerNext << 6) | (level & 0x3F)) & ~(1 << 7));
      
      // reset the marker
      sendMarkerNext = 0;
      // rest buffer;
      dmaBuffer[i];
    }
  }
  // get next sensor in line to convert from;
  //currentSensor = nextSensor(currentSensor);
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

  //DMA2->LISR |= 0x20;

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

  // set number of conversions in one sequence to 3, for 3 sensors;
  DMA2_Stream0->NDTR |= MAX_SENSORS;

  // set memory increment on, so that it will fill the buffer/array correctly;
  DMA2_Stream0->CR |= DMA_SxCR_MINC;

  // set DMA to circular mode so it will refill the increment once its empty;
  DMA2_Stream0->CR |= DMA_SxCR_CIRC;

  //
  //DMA2_Stream0->CR |= (1 << 3);

  // after all configurations are done, set the enable bit;
  DMA2_Stream0->CR |= DMA_SxCR_EN;
}

void configureADC(bool DMA)
{ 
  // initialise sensors;
  for (int i = 0; i < MAX_SENSORS; i++)
  {
    // set sensor input channel in sequence registers;
    ADC1->SQR3 |= (i << (i * 5));

    // set pins to analog input in the GPIO mode register;
    GPIOA->MODER |= (0x3 << (i * 2));
  }

  // set amount of conversions to 3 (0 = 1 conversion);
  ADC1->SQR1 |= ((MAX_SENSORS - 1) << 20);

  // set Resolution bits to 10 bit resolution;
  ADC1->CR1 |= 0x01000000;

  // highest conversion time
  ADC1->SMPR2 |= 0x3FF;

  // enable scan mode to scan for next channel for conversion
  ADC1->CR1 |= ADC_CR1_SCAN;

  if (DMA)
  {
    // DMA requests are issued as long as data are converted and DMA=1;
    ADC1->CR2 |= (1 << 9); // DDS bit
    
    configureDMA();

    // enable the DMA in the ADC after the DMA is configured;
    ADC1->CR2 |= ADC_CR2_DMA;
  }
  // set ADON bit in ADC control register to turn the converter on;
  ADC1->CR2 |= ADC_CR2_ADON; 

    // enable continuous mode
  ADC1->CR2 |= ADC_CR2_CONT;

 // set ADON bit in ADC control register for the second time to actually turn the converter on;
  ADC1->CR2 |= ADC_CR2_ADON; 
}

void configureSensors(boolean init)
{
  // configure sensors from EEPROM?
  if (init)
  {
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
  else 
  {
    configureFromEEEPROM();
  }

}

//uint16_t x = 0;

//void DMA2_Stream0_IRQHandler (void) {
  //NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
  //x = 1;
//}

void setup()
{
  // baudrate 4M for development;
  Serial.begin(4000000);

  // enable ADC system clock;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // enable DMA system clock;
  RCC->AHB1ENR |= 0x400000;

  configureADC(true);

  //HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 3);
  
  //__disable_irq();

  //HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  //uint32_t temp = NVIC_GetPriority(DMA2_Stream0_IRQn);
  //NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_GetPriority(OTG_FS_IRQn) + 1);
  //NVIC_SetPriority(OTG_FS_IRQn, temp);
  //NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  //SCB->VTOR for vector table address  
  //&isrhandler;
  //SCB->VTOR  


  //HAL_FLASH_Unlock();

  //EE_Init();
  
  // configure sensors;
  configureSensors(true);

  //NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  //__enable_irq();
  // set Start conversion bit in Control Register 2;
  ADC1->CR2 |= ADC_CR2_SWSTART;
}

void loop()
{ 
  //__disable_irq();
  //NVIC_DisableIRQ(DMA2_Stream0_IRQn);
  //__enable_irq();
  //Serial.print(1);
  //Serial.print(NVIC->ISER[2],BIN);
  //Serial.println();
  //Serial.println(x);
  //Serial.println(NVIC->ICER[1],BIN);
  //Serial.print(NVIC->ISPR[1],BIN);
  //Serial.println();
  //__disable_irq();
  //NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  // check if OVR bit is set in ADC status register;
  if (ADC1->SR & (1<<5))
  {
    // this should get another value;
    Serial.write(((3 & 0x7) << 4) | ((512 & 0x3C0) >> 6) | (1 << 7));
    Serial.write(((sendMarkerNext << 6) | (512 & 0x3F)) & ~(1 << 7));  
  }
  else 
  {
    // check if the conversion has ended;
    if (conversionComplete())
    {
      ADC_Handler();
    }
  }
  // manually check if there is input from the host;
  serialEvent();
  
}
