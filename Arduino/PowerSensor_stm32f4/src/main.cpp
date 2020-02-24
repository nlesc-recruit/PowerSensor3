#include <Arduino.h>

// includes libmaple;
#include <scb.h>
#include <adc.h>
#include <gpio.h>

// defines;
#define MAX_SENSORS 5

bool streamValues = false;
uint8_t sendMarkerNext = 0;

struct Sensor {
  bool inUse;
  uint16_t count;
  int32_t total;
  double weight;
  double nullLevel;
  uint8_t pin;

  double power() const { return weight * total / count - nullLevel; }

} sensors[MAX_SENSORS];

void configureSensors() 
{
  sensors[0].pin = PA4;
  sensors[0].inUse = true;
  sensors[1].pin = PA5;
  sensors[1].inUse = true;
  sensors[2].pin = PA6;
  sensors[2].inUse = true;
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
  // baudrate 9600 for development, upgrade to 2M later;
  Serial.begin(4000000); 

  // set vector table to the start of the flash;
  //nvic_init(0x08000000, 0);
  //SCB_BASE->VTOR = 0x08000000 | (0 & 0x1FFFFF80);

  // set vecor handler to ADC hander
  //nvic_irq_enable(NVIC_ADC_1_2);
  //NVIC_BASE->ISER[NVIC_ADC_1_2 / 32] |= BIT(NVIC_ADC_1_2 % 32);

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

  // set Start conversion bit in Control Register 2;
  ADC1_BASE->CR2 |= ADC_CR2_SWSTART;

  // configure sensors;
  configureSensors();
}

void loop() 
{
  // check if the End of Conversion bit is set in the ADC1 Status Register;
  if (ADC1_BASE->SR & ADC_SR_EOC) 
  {
    ADC_Handler();
  }
  
  // manually check if there is input from the host;
  serialEvent();
}

