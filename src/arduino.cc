#include <Arduino.h>


# define __ADC__
//# define __NVIC__

uint temreg1, temreg2;

void setup() 
{

  #if defined __ADC__
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC1->CR2 |= ADC_CR2_ADON; // Turn A/D converter on 
  //ADC1->CR1 = ADC_CR1_EOCIE; // interrupt enable for end of conversion
  //ADC1->SQR3 |= 1; // use PA04 as input
  #endif

  #if defined __NVIC__
  NVIC_BASE->ISER[NVIC_TIMER2 / 32] = BIT(NVIC_TIMER2 % 32); // enables TIM2
  #endif
  Serial.begin(9600);
  //ADC1->CR2 |= ADC_CR2_SWSTART;
}

void ADC_IRQHandler(void) 
{
  Serial.println("IRQ");
}

int running = 0;

void loop() 
{
  if(running) 
  {
    if(ADC1->SR & (1<<1))
    {
      temreg2 = ADC1->SR;
      Serial.print("temreg2: ");
      Serial.println(temreg2, BIN);
      uint16_t ainput = ADC1->DR; // get what is converted
      Serial.print("ainput: ");
      Serial.println(ainput);
      running = 0;
    }
  }
  else
  {
    delay(1000);
    ADC1->CR2 |= ADC_CR2_SWSTART; // start AD conversion
    temreg1 = ADC1->SR;
    Serial.print("temreg1: ");
    Serial.println(temreg1, BIN);
    running = 1;
  }
}
