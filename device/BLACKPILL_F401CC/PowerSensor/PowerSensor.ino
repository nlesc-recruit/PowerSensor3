#define USE_FULL_LL_DRIVER
#define MAX_SENSORS 8  // limited by number of bits used for sensor id
#define USE_DISPLAY  // comment out to disable display
#define VERSION "0.1.0"

// these two values are used to be able to jump to the bootloader from the application
#define SYSMEM_RESET_VECTOR            0x1FFF0004
#define BOOTLOADER_STACK_POINTER       0x20002560

#include <Arduino.h>
#include <stm32f4xx_ll_bus.h>  // clock control
#include <stm32f4xx_ll_adc.h>  // ADC control
#include <stm32f4xx_ll_gpio.h> // GPIO control
#include <stm32f4xx_ll_dma.h>  // DMA control
#include <EEPROM.h>

#ifdef USE_DISPLAY
#define UPDATE_INVERVAL 2000  // ms
#define VOLTAGE 3.3
#define MAX_PAIRS 4
#define MAX_LEVEL 1023
#include "display.h"
uint16_t sensorLevels[MAX_SENSORS];  // to store raw (averaged) sensor values for displaying purposes
float voltageValues[MAX_SENSORS/2];
float currentValues[MAX_SENSORS/2];
float powerValues[MAX_SENSORS/2];
float totalPower;
bool displayEnabled = true;
#endif

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

const int numSampleToAverage = 6; // number of samples to average
uint32_t counter;
uint8_t numSensor;  // number of active sensors
int activeSensors[MAX_SENSORS]; // which sensors are active
int activeSensorPairs[MAX_SENSORS/2];
uint16_t dmaBuffer[MAX_SENSORS];  // 16b per sensor
uint16_t avgBuffer[MAX_SENSORS][numSampleToAverage];
uint16_t currentSample = 0;
bool streamValues = false;
bool sendSingleValue = false;
bool sendMarkerNext = false;

struct Sensor {
  char type[16];
  float vref;
  float sensitivity;
  bool inUse;
} __attribute__((packed));

struct Config {
  Sensor sensors[MAX_SENSORS];
} eeprom;

void JumpToBootloader() {
  // ensure DMA and ADC are off
  LL_ADC_Disable(ADC1);
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
  LL_ADC_DeInit(ADC1);
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_0);

 // function starting at to whatever address is stored in the reset vector
 void (*SysMemBootJump)(void) = (void (*)(void)) (*((uint32_t *) SYSMEM_RESET_VECTOR));
 HAL_DeInit();
 HAL_RCC_DeInit();
 SysTick->CTRL = 0;
 SysTick->LOAD = 0;
 SysTick->VAL = 0;
 __set_MSP(BOOTLOADER_STACK_POINTER);
 SysMemBootJump();
}

void readConfig() {
  // send config in virtual EEPROM to host in chunks per sensor
  // after each chunk, any character should be sent to the device to
  // trigger sending the next chunk. D is sent when done
  for (int s=0; s<MAX_SENSORS; s++) {
    Serial.write((const uint8_t*) &eeprom.sensors[s], sizeof(Sensor));
    while (Serial.read() < 0) {
    }
  }
  Serial.write('D');
}

void writeConfig() {
  // read eeprom from host per byte
  // in chunks per sensor
  // send S to host after each sensor, D when done
  uint8_t* p_eeprom = (uint8_t*) &eeprom;
  for (int s=0; s<MAX_SENSORS; s++) {
    // wait for entire sensor chunk to be available
    while (Serial.available() < sizeof(Sensor)) {
    }
    // write sensor bytes into eeprom struct
    for (int b=0; b<sizeof(Sensor); b++) {
      *p_eeprom++ = Serial.read();
    }
    Serial.write('S');
  }
  // store updated EEPROM data to flash
  writeEEPROMToFlash();
  // reconfigure the device
  configureDevice();
  // signal to host that device is ready
  Serial.write('D');
}

void readEEPROMFromFlash() {
  // copy from flash to buffer
  eeprom_buffer_fill();
  // read buffer per byte
  uint8_t* p_eeprom = (uint8_t*) &eeprom;
  for (uint16_t i=0; i < sizeof eeprom; i++) {
    *p_eeprom++ = eeprom_buffered_read_byte(i);
  }
}

void writeEEPROMToFlash() {
  // write buffer per byte
  uint8_t* p_eeprom = (uint8_t*) &eeprom;
  for (uint16_t i=0; i < sizeof eeprom; i++) {
    eeprom_buffered_write_byte(i, *p_eeprom++);
  }
  // copy from buffer to flash
  eeprom_buffer_flush();
}

void getActiveSensors() {
  numSensor = 0;
  for (int i=0; i < MAX_SENSORS; i++) {
    if (eeprom.sensors[i].inUse) {
      activeSensors[numSensor] = i;
      activeSensorPairs[numSensor/2] = i / 2;
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

  ADCCommonConfig.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;

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
    Blink(2);
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
    LL_ADC_SetChannelSamplingTime(ADC1, ADC_CHANNELS[sensor_id], LL_ADC_SAMPLINGTIME_3CYCLES);
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
    storeADCValues(/* store_only */ false);
  // if the display is enabled, make sure to always read out the values, but do not send them to host if not enabled
  #ifndef USE_DISPLAY
  }
  #else
  } else {
    storeADCValues(/* store_only */ true);
  }
  #endif
  // clear DMA TC flag
  LL_DMA_ClearFlag_TC0(DMA2);
}

void storeADCValues(const bool store_only) {
  // loop over sensors and store each value at current location in averaging buffer
  for (uint8_t i = 0; i < numSensor; i++) {
    avgBuffer[i][currentSample] = dmaBuffer[i];
  }
  currentSample++;
  // if the buffer is full, send the values and reset
  if (currentSample >= numSampleToAverage) {
    sendADCValues(store_only);
    currentSample = 0;
  }
}

void sendADCValues(const bool store_only) {
  // send all values over serial
  uint8_t data[numSensor*2];  // 2 bytes per sensor
  for (uint8_t i = 0; i < numSensor; i++) {
    uint8_t sensor_id = activeSensors[i];
    // calculate average level of current sensor
    uint16_t level = 0.;
    for (uint8_t j = 0; j < numSampleToAverage; j++) {
      level += avgBuffer[i][j];
    }
    level /= numSampleToAverage;
#ifdef USE_DISPLAY
    if (displayEnabled) {
      // store in sensorValues for display purposes
      sensorLevels[sensor_id] = level;
    }
#endif
    // add metadata to remaining bits: 2 bytes available with 10b sensor value
    // First byte: 1 iii aaaa
    // where iii is the sensor id, a are the upper 4 bits of the level
    data[2*i] = ((sensor_id & 0x7) << 4) | ((level & 0x3C0) >> 6) | (1 << 7);
    // Second byte: 0 m bbbbbb
    // where m is the marker bit, b are the lower 6 bits of the level
    data[2*i+1] = ((sendMarkerNext << 6) | (level & 0x3F)) & ~(1 << 7);
    counter++;
    sendMarkerNext = false;
  }
  sendSingleValue = false;
  if (store_only)
    return;
  Serial.write(data, sizeof data); // send data of all active sensors to host
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
      delay(100);  // on an RPi host the stop does not arrive at the host properly unless there is a delay here
      Serial.write((const uint8_t []) { 0xFF, 0x3F}, 2);
      Serial.write((const uint8_t []) { 0xFF, 0x3F}, 2);
      break;
    case 'Q':
      // Send value of internal counter of number of completed conversions. Used for testing and debugging
      Serial.write((const uint8_t*) &counter, sizeof counter);
      break;
    case 'B':
      // Blink
      Blink(1);
      break;
    case 'V':
      // Send firmware version in human-readable format
      Serial.print("Firmware version: ");
      Serial.println(VERSION);
      break;
    case 'Z':
      // Reset device
      NVIC_SystemReset();
      break;
    case 'Y':
      // Reset device to bootloader, enables DFU mode
      JumpToBootloader();
      break;
#ifdef USE_DISPLAY
    case 'D':
      // toggle display
      displayEnabled = not displayEnabled;
      if (displayEnabled) {
        initDisplay();
      } else {
        deinitDisplay();
      }
      break;
#endif
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


#ifdef USE_DISPLAY
void updateCalibratedSensorValues() {
  totalPower = 0;
  for (int pair=0; pair < MAX_SENSORS / 2; pair++) {
    float amp = (VOLTAGE * sensorLevels[2 * pair] / MAX_LEVEL - eeprom.sensors[2 * pair].vref) / eeprom.sensors[2 * pair].sensitivity;
    float volt = (VOLTAGE * sensorLevels[2 * pair + 1] / MAX_LEVEL - eeprom.sensors[2 * pair + 1].vref) / eeprom.sensors[2 * pair + 1].sensitivity;
    float power = volt * amp;
    voltageValues[pair] = volt;
    currentValues[pair] = amp;
    powerValues[pair] = power;
    totalPower += power;
  }
}

void updateDisplay() {
  static unsigned long previousMillis = 0;
  unsigned long interval = (unsigned long)(millis() - previousMillis);

  if (interval > UPDATE_INVERVAL) {
    static int sensor_pair = 0;
    previousMillis = millis();
    // clear the display by rewriting old values in the background color
    displaySensor(activeSensorPairs[sensor_pair], currentValues[sensor_pair], voltageValues[sensor_pair], powerValues[sensor_pair], totalPower, /* clearDisplay */ true);
    // update the values, then write to display
    sensor_pair = (sensor_pair + 1) % (numSensor / 2);
    updateCalibratedSensorValues();
    displaySensor(activeSensorPairs[sensor_pair], currentValues[sensor_pair], voltageValues[sensor_pair], powerValues[sensor_pair], totalPower);
  }
}
#endif


void setup() {
  Serial.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // HIGH is off

  // read virtual EEPROM data
  readEEPROMFromFlash();

  // enable clocks
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  // configure hardware (GPIO, DMA, ADC)
  configureDevice();
#ifdef USE_DISPLAY
  if (displayEnabled) {
    initDisplay();
  }
#endif
  Blink(1);
}

void loop() {
  // only check for serial events, sending sensor values to host is handled through interrupts
  serialEvent();
  // update display if enabled
#ifdef USE_DISPLAY
  if (displayEnabled) {
    updateDisplay();
  }
#endif
}
