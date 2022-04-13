#define USE_FULL_LL_DRIVER
#define MAX_SENSORS 8  // limited by number of bits used for sensor id
#define EEPROM_BASE_ADDRESS 0x1111

#include <Arduino.h>
#include <stm32f4xx_ll_bus.h>  // clock control
#include <stm32f4xx_ll_adc.h>  // ADC control
#include <stm32f4xx_ll_gpio.h> // GPIO control
#include <stm32f4xx_ll_dma.h>  // DMA control
#include "eeprom.h"
uint32_t counter;

const uint32_t ADC_SCANMODES[] = {LL_ADC_REG_SEQ_SCAN_DISABLE, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS,
                                  LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS,
                                  LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS, LL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS,
                                  LL_ADC_REG_SEQ_SCAN_ENABLE_7RANKS, LL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS};

const uint32_t ADC_RANKS[] = {LL_ADC_REG_RANK_1, LL_ADC_REG_RANK_2, LL_ADC_REG_RANK_3, LL_ADC_REG_RANK_4,
                              LL_ADC_REG_RANK_5, LL_ADC_REG_RANK_6, LL_ADC_REG_RANK_7, LL_ADC_REG_RANK_8};

const uint32_t ADC_CHANNELS[] = {LL_ADC_CHANNEL_0, LL_ADC_CHANNEL_1, LL_ADC_CHANNEL_2, LL_ADC_CHANNEL_3,
                                 LL_ADC_CHANNEL_4, LL_ADC_CHANNEL_5, LL_ADC_CHANNEL_6, LL_ADC_CHANNEL_7};

const uint32_t GPIO_PINS[] = {LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
                              LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7};

uint8_t numSensor;  // number of active sensors
int activeSensors[MAX_SENSORS]; // which sensors are active
uint16_t dmaBuffer[MAX_SENSORS];  // 16b per sensor
bool streamValues = false;
bool sendSingleValue = false;
bool sendMarkerNext = false;

struct Sensor {
  char type[16];
  float vref;
  float slope;
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
  // Blink LED, note that outputs are inverted: LOW is on, HIGH is off
  for (uint8_t i = 0; i < amount; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
  }
}

void configureADCCommon() {
  LL_ADC_CommonInitTypeDef ADCCommonConfig;
  LL_ADC_CommonStructInit(&ADCCommonConfig);

  ADCCommonConfig.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV8;

  // Apply settings
  if (LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADCCommonConfig) != SUCCESS) {
    Blink(1);
    exit(1);
  }
}

void configureADC() {
  LL_ADC_InitTypeDef ADCConfig;
  LL_ADC_StructInit(&ADCConfig);

  ADCConfig.Resolution = LL_ADC_RESOLUTION_10B; // 10-bit ADC resolution
  ADCConfig.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT; // right-align data within 16b register
  ADCConfig.SequencersScanMode = ADC_SCANMODES[numSensor - 1] == LL_ADC_REG_SEQ_SCAN_DISABLE ? LL_ADC_SEQ_SCAN_DISABLE: LL_ADC_SEQ_SCAN_ENABLE;  // enable scan only if there is more than one rank to convert

  if (LL_ADC_Init(ADC1, &ADCConfig) != SUCCESS) {
    Blink(1);
    exit(1);
  }
}

void configureADCChannels() {
  LL_ADC_REG_InitTypeDef ADCChannelConfig;
  LL_ADC_REG_StructInit(&ADCChannelConfig);

  ADCChannelConfig.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;   // trigger conversion from software
  ADCChannelConfig.SequencerLength = ADC_SCANMODES[numSensor - 1];  // number of ranks to convert
  ADCChannelConfig.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE; // not using discontinous mode
  ADCChannelConfig.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS; // enable continuous conversion mode
  ADCChannelConfig.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED; // Allow unlimited transfers to DMA

  if (LL_ADC_REG_Init(ADC1, &ADCChannelConfig) != SUCCESS) {
    Blink(3);
    exit(1);
  }

  // Set which channels will be converted and in which order, as well as their sampling time
  for (uint8_t i = 0; i < numSensor; i++) {
    uint8_t sensor_id = activeSensors[i];
    LL_ADC_REG_SetSequencerRanks(ADC1, ADC_RANKS[i], ADC_CHANNELS[sensor_id]);
    LL_ADC_SetChannelSamplingTime(ADC1, ADC_CHANNELS[sensor_id], LL_ADC_SAMPLINGTIME_15CYCLES);
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

  DMAConfig.PeriphOrM2MSrcAddress =  LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA);
  DMAConfig.MemoryOrM2MDstAddress = (uint32_t) &dmaBuffer; // target is the buffer in RAM
  DMAConfig.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMAConfig.Mode = LL_DMA_MODE_CIRCULAR;
  DMAConfig.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMAConfig.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMAConfig.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  DMAConfig.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  DMAConfig.NbData = numSensor;
  DMAConfig.Channel = LL_DMA_CHANNEL_0;

  // write config
  if (LL_DMA_Init(DMA2, LL_DMA_STREAM_0, &DMAConfig) != SUCCESS) {
    Blink(5);
    exit(1);
  }

  // enable interrupt on transfer-complete
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);

  // enable the DMA stream
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
}

void configureNVIC() {
  // set the DMA interrupt to be lower than USB to avoid breaking communication to host
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_GetPriority(OTG_FS_IRQn) + 1);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

extern "C" void DMA2_Stream0_IRQHandler() {
  // send ADC values to host if enabled
  if (streamValues | sendSingleValue) {
    sendADCValue();
  }
  // clear DMA TC flag
  LL_DMA_ClearFlag_TC0(DMA2);
}

void sendADCValue() {
  // send all values over serial
  uint8_t data[numSensor*2];  // 2 bytes per sensor
  for (uint8_t i = 0; i < numSensor; i++) {
    uint8_t sensor_id = activeSensors[i];
    // pointer to level of current sensor
    uint16_t* level = dmaBuffer + i;
    // add metadata to remaining bits: 2 bytes available with 10b sensor value
    // First byte: 1 iii aaaa
    // where iii is the sensor id, a are the upper 4 bits of the level
    data[2*i] = ((sensor_id & 0x7) << 4) | ((*level & 0x3C0) >> 6) | (1 << 7);
    // Second byte: 0 m bbbbbb
    // where m is the marker bit, b are the lower 6 bits of the level
    data[2*i+1] = ((sendMarkerNext << 6) | (*level & 0x3F)) & ~(1 << 7);
    counter++;
    sendMarkerNext = false;
  }
  Serial.write(data, sizeof data); // send data of all active sensors to host
  sendSingleValue = false;
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
    case 'M':
      // marker character, places a marker in the output file
      sendMarkerNext = true;
    case 'I':
      // Send single set of sensor values. does nothing if streaming is enabled
      sendSingleValue = true;
      break;
    case 'S':
      // Enable streaming of data
      counter = 0;
      streamValues = true;
      break;
    case 'T':
      // Disable streaming of data
      streamValues = false;
      break;
    case 'X':
      // Shutdown, shuts off IO thread on host
      streamValues = false;
      Serial.write((const uint8_t []) { 0xFF, 0x3F}, 2);
      Serial.write((const uint8_t []) { 0xFF, 0x3F}, 2);
    case 'Q':
      // Send value of internal counter of number of completed conversions. Used for testing and debugging
      Serial.write((const uint8_t*) &counter, sizeof counter);
      break;
   }
  }
}

void configureDevice() {
  // ensure DMA and ADC are off
  LL_ADC_Disable(ADC1);
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);

  getActiveSensors();
  configureGPIO();
  configureDMA();
  configureADCCommon();
  configureADC();
  configureADCChannels();
  configureNVIC();
  LL_ADC_Enable(ADC1);
  LL_ADC_REG_StartConversionSWStart(ADC1);
}

void setup() {
  Serial.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // HIGH is off

  // Setup and read virtual EEPROM data
  generateVirtualAddresses();
  HAL_FLASH_Unlock();
  EE_Init();
  readEEPROMFromFlash();

  // enable clocks
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  // configure hardware (GPIO, DMA, ADC)
  configureDevice();
}

void loop() {
  // only check for serial events, sending sensor values to host is handled through interrupts
  serialEvent();
}
