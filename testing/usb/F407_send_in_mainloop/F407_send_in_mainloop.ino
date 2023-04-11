#define USE_FULL_LL_DRIVER
#define SENSORS 2
#define PAIRS 1
#define VERSION "TEST-F407-0.1"
//#define INTERACTIVE

#include <Arduino.h>
#include <stm32f4xx_ll_bus.h>  // clock control
#include <stm32f4xx_ll_adc.h>  // ADC control
#include <stm32f4xx_ll_gpio.h> // GPIO control
#include <stm32f4xx_ll_dma.h>  // DMA control

const uint32_t ADC_SCANMODES[] = {LL_ADC_REG_SEQ_SCAN_DISABLE, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS,
                                  LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS};

const uint32_t ADC_RANKS[] = {LL_ADC_REG_RANK_1, LL_ADC_REG_RANK_2, LL_ADC_REG_RANK_3, LL_ADC_REG_RANK_4};

const uint32_t ADC_CHANNELS[] = {LL_ADC_CHANNEL_0, LL_ADC_CHANNEL_1, LL_ADC_CHANNEL_2, LL_ADC_CHANNEL_3,
                                 LL_ADC_CHANNEL_4, LL_ADC_CHANNEL_5, LL_ADC_CHANNEL_6, LL_ADC_CHANNEL_7};

const uint32_t GPIO_PINS[] = {LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
                              LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7};


ADC_TypeDef* ADCCurrent = ADC1;  // use first ADC for current sensors
ADC_TypeDef* ADCVoltage = ADC2; // use second ADC for voltage sensors

uint8_t numSensor = SENSORS;
uint32_t dmaBuffer;  // DMA reads both ADCs at the same time to one 32b value
uint32_t counter = 0;
uint32_t tstart;
uint32_t tend;
bool measurement_running = false;

uint8_t data[6];
bool sendData = false;
uint32_t missed = 0;

/*
 * Simple counter; Rate = 159 KHz
 */
//extern "C" void DMA2_Stream0_IRQHandler() {
//  // keep track of number of conversions
//  counter++;
//  // clear DMA TC flag
//  LL_DMA_ClearFlag_TC0(DMA2);
//}

extern "C" void DMA2_Stream0_IRQHandler() {
  // extract the two ADC values
  uint16_t level_current = __LL_ADC_MULTI_CONV_DATA_MASTER_SLAVE(LL_ADC_MULTI_MASTER, dmaBuffer);
  uint16_t level_voltage = __LL_ADC_MULTI_CONV_DATA_MASTER_SLAVE(LL_ADC_MULTI_SLAVE, dmaBuffer);
  uint16_t dt = micros();

  processValues(level_current, level_voltage, dt);

  // keep track of number of conversions
  counter++;
  // clear DMA TC flag
  LL_DMA_ClearFlag_TC0(DMA2);
}

void processValues(uint16_t level_current, uint16_t level_voltage, uint16_t dt) {
  // we send two timestamp (dt) bytes and two bytes per sensor
  // timestamp packet 0: 110 TTTTT (most significant 5 bits)
  // timestamp packet 1: 111 TTTTT (least significant 5 bits)
  // then repeat:
  // sensor 0 packet 0: 000 SSSSS (least sig bits)
  // sensor 0 packet 1: 001 SSSSS (most sig bits)
  // sensor 1 packet 0: 010 SSSSS
  // sensor 1 packet 1: 011 SSSSS
  // So first 3 bits are sensor id followed by 0 or 1 for the 2 packets
  // timestamp is "sensor" 11
  // timestamp
//  uint32_t tnew = micros();
//  static uint32_t t = tnew;  // ensure that the first dt is zero
//  uint16_t dt = tnew - t;
//  uint32_t dt = micros();

  // timestamp
  data[0] = (0b110 << 5) | ((dt >> 5) & 0x1F);
  data[1] = (0b111 << 5) | (dt & 0x1F);
  // sensor 0 (current)
  data[2] = (0b000 << 5) | ((level_current >> 5) & 0x1F);
  data[3] = (0b001 << 5) | (level_current & 0x1F);
  // sensor 1 (voltage)
  data[4] = (0b010 << 5) | ((level_voltage >> 5) & 0x1F);
  data[5] = (0b011 << 5) | (level_voltage & 0x1F);

#ifndef INTERACTIVE
  if (measurement_running) {
    // if sendData is already true, the previous value has not been sent yet
    if (sendData) {
      missed++;
      return;  // skip t = tnew to not skew dt value
    }
    sendData = true;
//    Serial.write(data, sizeof(data));
  }
#endif
  // bookkeeping
//  t = tnew;
}

void Blink(uint8_t amount) {
  // Blink LED
  for (uint8_t i = 0; i < amount; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
}

void configureADCCommon() {
  LL_ADC_CommonInitTypeDef ADCCommonConfig;
  LL_ADC_CommonStructInit(&ADCCommonConfig);
  /*
   Data Fields
   • uint32_t CommonClock
   • uint32_t Multimode
   • uint32_t MultiDMATransfer
   • uint32_t MultiTwoSamplingDelay
  */
  ADCCommonConfig.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV8;  // at default cpu/bus clock speeds and a divider of 8, the ADC runs at 10.5 MHz
  ADCCommonConfig.Multimode = LL_ADC_MULTI_DUAL_REG_SIMULT;  // regular simultaneous mode
  ADCCommonConfig.MultiDMATransfer = LL_ADC_MULTI_REG_DMA_UNLMT_2; // allow unlimited DMA transfers. MODE2 = half-words by ADC pairs
  ADCCommonConfig.MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSMP_DELAY_5CYCLES; // fastest possible mode

  if (LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADCCurrent), &ADCCommonConfig) != SUCCESS) {
#ifdef INTERACTIVE
    Serial.println("Failed common ADC setup");
#else
    Blink(1);
#endif
  }
}

void configureADC(ADC_TypeDef* adc) {
  LL_ADC_InitTypeDef ADCConfig;
  LL_ADC_StructInit(&ADCConfig);
  /*
   Data Fields
   • uint32_t Resolution
   • uint32_t DataAlignment
   • uint32_t SequencersScanMode
  */
  ADCConfig.Resolution = LL_ADC_RESOLUTION_10B;
  ADCConfig.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADCConfig.SequencersScanMode = ADC_SCANMODES[(numSensor/2) - 1] == LL_ADC_REG_SEQ_SCAN_DISABLE ? LL_ADC_SEQ_SCAN_DISABLE: LL_ADC_SEQ_SCAN_ENABLE;  // enable scan only if there is more than one rank to convert

  if (LL_ADC_Init(adc, &ADCConfig) != SUCCESS) {
#ifdef INTERACTIVE
    Serial.println("Failed ADC setup");
#else
    Blink(1);
#endif
  }
}

void configureADCChannels(ADC_TypeDef* adc, const bool master) {
  LL_ADC_REG_InitTypeDef ADCChannelConfig;
  LL_ADC_REG_StructInit(&ADCChannelConfig);
  /*
   Data Fields
   • uint32_t TriggerSource
   • uint32_t SequencerLength
   • uint32_t SequencerDiscont
   • uint32_t ContinuousMode
   • uint32_t DMATransfer
  */
  ADCChannelConfig.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;   // trigger conversion from software
  ADCChannelConfig.SequencerLength =  ADC_SCANMODES[(numSensor/2) - 1];  // number of ranks to convert
  ADCChannelConfig.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE; // not using discontinous mode
  ADCChannelConfig.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS; // enable continuous conversion mode
  ADCChannelConfig.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED; // Allow unlimited transfers to DMA

  if (LL_ADC_REG_Init(adc, &ADCChannelConfig) != SUCCESS) {
#ifdef INTERACTIVE
    Serial.println("Failed ADC channel setup");
#else
    Blink(1);
#endif
  }

  // Set which channels will be converted and in which order, as well as their sampling time
  // allowed sampling times (in units of ADC clock cycles):
  // 3 15 28 56 84 112 114 480
  
  // Sampling time is set to be long enough to stay under 12 Mbps (max USB speed)
  // At 56 cycles, 10b resolution, 32 bits per sensor pair, and an ADC clock of 10.5 MHz, the data rate is
  // 10.5 MHz * 32 / (10 + 56) = 5.1 Mbps
  // sensors are sampled at
  // 10.5 MHz / (10+56) = 159.09 KHz -> 6.29 us
  //
  // with 144 cycles, sampling is 68.18 KHz -> 14.67 us
  // with 480 cycles, sampling is 21.43 KHz -> 46.67 us
  for (uint8_t i = !master; i < numSensor; i+=2) {
    uint8_t sensor_id = i;
    LL_ADC_REG_SetSequencerRanks(adc, ADC_RANKS[i/2], ADC_CHANNELS[sensor_id]);
    LL_ADC_SetChannelSamplingTime(adc, ADC_CHANNELS[sensor_id], LL_ADC_SAMPLINGTIME_144CYCLES);
  }
}

void configureDMA() {
  LL_DMA_InitTypeDef DMAConfig;
  LL_DMA_StructInit(&DMAConfig);

  DMAConfig.PeriphOrM2MSrcAddress =  LL_ADC_DMA_GetRegAddr(ADCCurrent, LL_ADC_DMA_REG_REGULAR_DATA_MULTI);  // macro handles obtaining common data register for multi-ADC mode
  DMAConfig.MemoryOrM2MDstAddress = (uint32_t) &dmaBuffer; // target is the buffer in RAM
  DMAConfig.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMAConfig.Mode = LL_DMA_MODE_CIRCULAR;
  DMAConfig.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMAConfig.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMAConfig.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMAConfig.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMAConfig.NbData = numSensor / 2;
  DMAConfig.Channel = LL_DMA_CHANNEL_0;

  if (LL_DMA_Init(DMA2, LL_DMA_STREAM_0, &DMAConfig) != SUCCESS) {
#ifdef INTERACTIVE
    Serial.println("Failed DMA setup");
#else
    Blink(1);
#endif
  }

  // enable interrupt on transfer-complete
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);

  // enable the DMA stream
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
}

void configureGPIO() {
  LL_GPIO_InitTypeDef GPIOConfig;
  LL_GPIO_StructInit(&GPIOConfig);

  uint32_t pins = 0;
  for (uint8_t i = 0; i < numSensor; i++) {
    uint8_t sensor_id = i;
    pins |= GPIO_PINS[sensor_id];
  }

  GPIOConfig.Pin = pins;
  GPIOConfig.Mode = LL_GPIO_MODE_ANALOG;

  if (LL_GPIO_Init(GPIOA, &GPIOConfig) != SUCCESS) {
#ifdef INTERACTIVE
    Serial.println("Failed GPIO setup");
#else
    Blink(1);
#endif
  }
}

void configureNVIC() {
  // set the DMA interrupt to be lower than USB to avoid breaking communication to host
//  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_GetPriority(OTG_FS_IRQn) + 1);
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_GetPriority(OTG_FS_IRQn) - 1);
//  NVIC_SetPriority(DMA2_Stream0_IRQn, 0);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void configureDevice() {
  // ensure DMA and ADC are off
  LL_ADC_Disable(ADCCurrent);
  LL_ADC_Disable(ADCVoltage);
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);

  configureGPIO();
  configureDMA();
  configureADCCommon();
  configureADC(ADCCurrent);
  configureADC(ADCVoltage);
  configureADCChannels(ADCCurrent, /* master */ true);
  configureADCChannels(ADCVoltage, /* master */ false);
  configureNVIC();
  LL_ADC_Enable(ADCCurrent);
  LL_ADC_Enable(ADCVoltage);
  LL_ADC_REG_StartConversionSWStart(ADCCurrent);
}

void process() {
  uint32_t dt = tend - tstart;
  double rate = static_cast<double>(counter) / dt;  // samples per ms = kHz
  Serial.print("Time passed (ms): ");
  Serial.println(dt);

  Serial.print("Number of samples: ");
  Serial.println(counter);

  Serial.print("Rate: ");
  Serial.println(rate);
//  Serial.println("Expected rate is 159.09 KHz");
//  Serial.println("Expected data rate at 16b/sensor is 5.09 Mbps");
}

void serialEvent() {
  if (Serial.available() > 0) {
    switch (Serial.read()) {
    case 'S':
      // Start
#ifdef INTERACTIVE
      Serial.println("Starting measurement");
#endif
      measurement_running = true;
      tstart = millis();
      counter = 0;
      break;
    case 'T':
      // Stop
      measurement_running = false;
      tend = millis();
#ifdef INTERACTIVE
      Serial.println("Stopping measurement");
      process();
#endif
      break;
    case 'M':
      Serial.print("Missed values: ");
      Serial.println(missed);
      break;
    case '\n':
      return;
    default:
#ifdef INTERACTIVE
      Serial.println("Ignoring unknown command");
#endif
      break;
    }
  }
}

void setup() {
  Serial.begin();
  pinMode(LED_BUILTIN, OUTPUT);

#ifdef INTERACTIVE
  // Start when signal from host is received
  Blink(1);
  while (Serial.available() == 0) {}
  Serial.read();
  Serial.println("Starting setup");
#endif

  // enable clocks
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

  // configure hardware (GPIO, DMA, ADC)
  configureDevice();

#ifdef INTERACTIVE
  Serial.println("Finished setup");
#else
  Blink(1);
#endif
}

void loop() {
  serialEvent();
  if (sendData) {
    sendData = false;
    Serial.write(data, sizeof(data));
  }
}
