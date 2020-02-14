#include <Arduino.h>

// define to test sample speeds locally
// #define __TIMING__

#if defined __TIMING__
uint32_t counter = 0;
ulong time;
#endif

void ADC_Handler(void)
{
  // read out the Data Register of ADC1;
  uint32_t level = ADC1_BASE->DR;
  
  // write the level, write() only writes per byte;
  Serial.write(level << 24);
  Serial.write(level << 16);
  Serial.write(level << 8);
  Serial.write(level);

  #if defined __TIMING__
  // add 1 sample to the counter;
  counter++;
  #endif

  // set Start conversion bit in Control Register 2;
  ADC1_BASE->CR2 |= ADC_CR2_SWSTART;
}

void setup() 
{
  // baudrate 9600 for development, upgrade to 2M later;
  Serial.begin(8000000); 

  // set ADON bit in ADC control register to turn the converter on;
  ADC1_BASE->CR2 |= ADC_CR2_ADON; 

  // set PA4 in sequence register 3 to be input for conversion;
  ADC1_BASE->SQR3 |= PA4;

  // set PA4 to analog input in the GPIO mode register;
  GPIOA_BASE->MODER |= 0x00000300;

  // set End of Conversion Interrupt Enable bit to enable interrupts;
  ADC1_BASE->CR1 |= ADC_CR1_EOCIE;

  // enable IRQ for ADC channels 1 and 2;
  // NVIC_BASE->ISER[(((uint32_t)(int32_t)NVIC_ADC_1_2) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)NVIC_ADC_1_2) & 0x1FUL)); SCB_BASE->VTOR???

  // set Start conversion bit in Control Register 2;
  ADC1_BASE->CR2 |= ADC_CR2_SWSTART;
  delay(3000);

  #if defined __TIMING__
  time = micros();
  #endif

}

uint running = 1;

void loop() 
{
  while (running)
  {
    // check if the End of Conversion bit is set in the ADC1 Status Register;
    if (ADC1_BASE->SR & ADC_SR_EOC) 
    {
      ADC_Handler();
    }
    
    #if defined __TIMING__
    // if one second has elapsed, print the amount of samples taken;
    if ((micros() - time) > 1000000) 
    {
      Serial.println("1 s elapsed");
      Serial.println(counter);
      counter = 0;
      time = micros();
      running = 0;
    }
    #endif
  }
}
