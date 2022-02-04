#define USE_FULL_LL_DRIVER
#define MAX_SENSORS 8  // limited by number of bits used for sensor id
#define EEPROM_BASE_ADDRESS 0x1111

#include <Arduino.h>
#include <stm32f4xx_ll_bus.h>  // clock control
#include <stm32f4xx_ll_adc.h>  // ADC control
#include <stm32f4xx_ll_gpio.h> // GPIO control
#include <stm32f4xx_ll_dma.h>  // DMA control
#include "eeprom.h"

const uint32_t ADC_SCANMODES[] = {LL_ADC_REG_SEQ_SCAN_DISABLE, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS,
                                  LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS};

const uint32_t ADC_RANKS[] = {LL_ADC_REG_RANK_1, LL_ADC_REG_RANK_2, LL_ADC_REG_RANK_3, LL_ADC_REG_RANK_4};

const uint32_t ADC_CHANNELS[] = {LL_ADC_CHANNEL_0, LL_ADC_CHANNEL_1, LL_ADC_CHANNEL_2, LL_ADC_CHANNEL_3,
                                 LL_ADC_CHANNEL_4, LL_ADC_CHANNEL_5, LL_ADC_CHANNEL_6, LL_ADC_CHANNEL_7};

const uint32_t GPIO_PINS[] = {LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
                              LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7};

ADC_TypeDef* ADCCurrent = ADC1;  // use first ADC for current sensors
ADC_TypeDef* ADCVoltage = ADC2; // use second ADC for voltage sensors
uint8_t numSensor;  // number of active sensors (in total, not per ADC)
int activeSensors[MAX_SENSORS]; // which sensors are active
uint32_t dmaBuffer[MAX_SENSORS / 2];  // DMA reads both ADCs at the same time to one 32b value
uint16_t serialBuffer[MAX_SENSORS];  // data sent to host is 16b per sensor
bool streamValues = false;
bool sendSingleValue = false;
bool sendMarkerNext = 0;

struct Sensor {
  char type[16];
  float vref;
  float slope;
  uint8_t pairId;
  bool inUse;
} __attribute__((packed));

struct EEPROM {
  Sensor sensors[MAX_SENSORS];
} eeprom;

const uint16_t eepromSize = sizeof(eeprom) / 2;  // size of EEPROM in half-words
uint16_t VirtAddVarTab[eepromSize];

void generateVirtualAddresses() {
  uint16_t addr = EEPROM_BASE_ADDRESS;
  for (int i = 0; i < eepromSize; i++) {
    VirtAddVarTab[i] = addr++;
  }
}

void readConfig() {
  // send config in virtual EEPROM to host
  Serial.write((const uint8_t*) &eeprom, sizeof eeprom);
}

void writeConfig() {
  // read config from host and write to virtual EEPROM
  uint8_t* p_eeprom = (uint8_t*) &eeprom;
  for (int i=0; i < sizeof eeprom; i++) {
    while (Serial.available() == 0) {
    }
    p_eeprom[i] = Serial.read();
  }
  // store updated EEPROM data to flash
  writeEEPROMToFlash();
  // reconfigure the device
  configureDevice();
}

void readEEPROMFromFlash() {
  // read per half-word
  uint16_t* p_eeprom = (uint16_t*) &eeprom;
  for (int i=0; i < eepromSize; i++) {
    EE_ReadVariable(VirtAddVarTab[i], p_eeprom);
    p_eeprom++;
    delay(1);
  }
}

void writeEEPROMToFlash() {
  // write per half-word
  uint16_t* p_eeprom = (uint16_t*) &eeprom;
  for (int i=0; i < eepromSize; i++) {
    EE_WriteVariable(VirtAddVarTab[i], *p_eeprom);
    p_eeprom++;
    delay(1);
  }
}

void getActiveSensors() {
  numSensor = 0;
  for (int i=0; i < MAX_SENSORS; i++) {
    if (eeprom.sensors[i].inUse) {
      activeSensors[numSensor] = i;
      numSensor++;
    }
  }
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

  ADCCommonConfig.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;  // at default cpu/bus clock speeds and a divider of 4, the ADC runs at 21 MHz
  ADCCommonConfig.Multimode = LL_ADC_MULTI_DUAL_REG_SIMULT;  // regular simultaneous mode
  ADCCommonConfig.MultiDMATransfer = LL_ADC_MULTI_REG_DMA_UNLMT_2; // allow unlimited DMA transfers. MODE2 = half-words by ADC pairs
  ADCCommonConfig.MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSMP_DELAY_5CYCLES; // fastest possible mode

  // Apply settings
  if (LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADCCurrent), &ADCCommonConfig) != SUCCESS) {
    Blink(1);
    exit(1);
  }
}

void configureADC(ADC_TypeDef* adc) {
  LL_ADC_InitTypeDef ADCConfig;
  LL_ADC_StructInit(&ADCConfig);

  ADCConfig.Resolution = LL_ADC_RESOLUTION_10B; // 10-bit ADC resolution
  ADCConfig.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT; // right-align data within 16b register
  ADCConfig.SequencersScanMode = ADC_SCANMODES[(numSensor/2) - 1] == LL_ADC_REG_SEQ_SCAN_DISABLE ? LL_ADC_SEQ_SCAN_DISABLE: LL_ADC_SEQ_SCAN_ENABLE;  // enable scan only if there is more than one rank to convert

  if (LL_ADC_Init(adc, &ADCConfig) != SUCCESS) {
    Blink(1);
    exit(1);
  }
}

void configureADCChannels(ADC_TypeDef* adc, bool master) {
  LL_ADC_REG_InitTypeDef ADCChannelConfig;
  LL_ADC_REG_StructInit(&ADCChannelConfig);

  ADCChannelConfig.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;   // trigger conversion from software
  ADCChannelConfig.SequencerLength = ADC_SCANMODES[(numSensor/2) - 1];  // number of ranks to convert
  ADCChannelConfig.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE; // not using discontinous mode
  ADCChannelConfig.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS; // enable continuous conversion mode
  ADCChannelConfig.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED; // Allow unlimited transfers to DMA

  if (LL_ADC_REG_Init(adc, &ADCChannelConfig) != SUCCESS) {
    Blink(3);
    exit(1);
  }

  // Set which channels will be converted and in which order, as well as their sampling time
  // Sampling time is set to be long enough to stay under 12 Mbps (max USB speed)
  // At 56 cycles, 10b resolution, 32 bits per sensor pair, and an ADC clock of 21 MHz, the data rate is
  // 21 MHz * 32 / (10 + 56) = 10.2 Mbps
  // With 4 sensor pairs, each pair is sampled at
  // 21 MHz / (10+56) / 4 = 79.5 KHz

  for (uint8_t i = !master; i < numSensor; i+=2) {
    uint8_t sensor_id = activeSensors[i];
    LL_ADC_REG_SetSequencerRanks(adc, ADC_RANKS[i/2], ADC_CHANNELS[sensor_id]);
    LL_ADC_SetChannelSamplingTime(adc, ADC_CHANNELS[sensor_id], LL_ADC_SAMPLINGTIME_56CYCLES);
  }

}

void configureGPIO() {
  // Create GPIO config struct and fill with defaults
  LL_GPIO_InitTypeDef GPIOConfig;
  LL_GPIO_StructInit(&GPIOConfig);

  uint32_t pins = 0;
  for (uint8_t i = 0; i < numSensor; i++) {
    uint8_t sensor_id = activeSensors[i];
    pins |= GPIO_PINS[sensor_id];
  }

  GPIOConfig.Pin = pins;
  GPIOConfig.Mode = LL_GPIO_MODE_ANALOG;

  if (LL_GPIO_Init(GPIOA, &GPIOConfig) != SUCCESS) {
    Blink(4);
    exit(1);
  }
}

void configureDMA() {
  // Create DMA config struct and fill with defaults
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

  // write config
  if (LL_DMA_Init(DMA2, LL_DMA_STREAM_0, &DMAConfig) != SUCCESS) {
    Blink(5);
    exit(1);
  }

  // enable the DMA stream
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
}

void sendADCValue() {
  if (streamValues | sendSingleValue) {
    // extract data of the two ADCs
    for (uint8_t i = 0; i < numSensor; i+=2) {
      serialBuffer[i] = __LL_ADC_MULTI_CONV_DATA_MASTER_SLAVE(LL_ADC_MULTI_MASTER, dmaBuffer[i/2]);
      serialBuffer[i+1] = __LL_ADC_MULTI_CONV_DATA_MASTER_SLAVE(LL_ADC_MULTI_SLAVE, dmaBuffer[i/2]);
    }

    // send all values over serial
    for (uint8_t i = 0; i < numSensor; i++) {
      uint8_t sensor_id = activeSensors[i];
      // add metadata to remaining bits: 2 bytes available with 10b sensor value
      uint16_t level = serialBuffer[i];
      // write the level, write() only writes per byte;
      // First byte: 1 iii aaaa
      // where iii is the sensor id, a are the upper 4 bits of the level
      Serial.write(((sensor_id & 0x7) << 4) | ((level & 0x3C0) >> 6) | (1 << 7));
      // Second byte: 0 m bbbbbb
      // where m is the marker bit, b are the lower 6 bits of the level
      Serial.write(((sendMarkerNext << 6) | (level & 0x3F)) & ~(1 << 7));
    }
    sendSingleValue = false;
  }
}

void serialEvent() {
  if (Serial.available() > 0) {
   switch (Serial.read()) {
    case 'R':
      // read sensor configuration from EEPROM
      readConfig();
      break;
    case 'W':
      // write sensor configuration to EEPROM
      writeConfig();
      break;
    case 'I':
      // Send single set of sensor values. does nothing if streaming is enabled
      sendSingleValue = true;
      break;
    case 'S':
      // Enable streaming of data
      streamValues = true;
      break;
    case 'T':
      // Disable streaming of data
      streamValues = false;
      break;
   }
  }
}

void configureDevice() {
  // ensure DMA and ADC are off
  LL_ADC_Disable(ADCCurrent);
  LL_ADC_Disable(ADCVoltage);
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);

  getActiveSensors();
  configureGPIO();
  configureDMA();
  configureADCCommon();
  configureADC(ADCCurrent);
  configureADC(ADCVoltage);
  configureADCChannels(ADCCurrent, true);
  configureADCChannels(ADCVoltage, false);
  LL_ADC_Enable(ADCCurrent);
  LL_ADC_Enable(ADCVoltage);
  LL_ADC_REG_StartConversionSWStart(ADCCurrent);
}

void setup() {
  Serial.begin(40000000);
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup and read virtual EEPROM data
  generateVirtualAddresses();
  HAL_FLASH_Unlock();
  EE_Init();
  readEEPROMFromFlash();

  // enable clocks
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

  // configure hardware (GPIO, DMA, ADC)
  configureDevice();
}

void loop() {
  // if a set of conversions has finished, transmit the values
  if (LL_DMA_IsActiveFlag_TC0(DMA2) == 1) {
    sendADCValue();
    // clear the transfer-complete flag
    LL_DMA_ClearFlag_TC0(DMA2);
  }

  serialEvent();
}
