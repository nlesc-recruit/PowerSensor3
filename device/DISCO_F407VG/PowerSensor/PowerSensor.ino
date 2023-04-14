/* NOTE: to avoid dropping data one must increase the USB transmit buffer size
 * See CDC_TRANSMIT_QUEUE_BUFFER_SIZE in packages/STMicroelectronics/hardware/stm32/2.3.0/cores/arduino/stm32/usb/cdc/cdc_queue.h
 * A value of 6 times CDC_TRANSMIT_QUEUE_BUFFER_SIZE instead of the default 2 times CDC_TRANSMIT_QUEUE_BUFFER_SIZE seems to be enough
 */

#define USE_FULL_LL_DRIVER
#define MAX_SENSORS 8  // limited by number of bits used for sensor id
#define USE_DISPLAY  // comment out to disable display
#define VERSION "F407-0.1.0"

// these two values are used to be able to jump to the bootloader from the application
// Start of system memory is 0x1FFF 0000, see Table 3. Memory mapping vs. Boot mode/physical remap in STM32F405xx/07xx and STM32F415xx/17xx
// in the reference manual.
// at boot the first word contains the initial value of the stack pointer. However this can change depending on the firmware.
// To be safe we put the stack at the end of the first RAM block. RAM starts at 0x2000 0000 and the first block is 12 KB in size
// the second word stores the address at which code execution should start, this is where we jump to to run the bootloader
#define SYSMEM_RESET_VECTOR            0x1FFF0004
#define BOOTLOADER_STACK_POINTER       0x2001BFFF

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
                                  LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS};

const uint32_t ADC_RANKS[] = {LL_ADC_REG_RANK_1, LL_ADC_REG_RANK_2, LL_ADC_REG_RANK_3, LL_ADC_REG_RANK_4};

const uint32_t ADC_CHANNELS[] = {LL_ADC_CHANNEL_0, LL_ADC_CHANNEL_1, LL_ADC_CHANNEL_2, LL_ADC_CHANNEL_3,
                                 LL_ADC_CHANNEL_4, LL_ADC_CHANNEL_5, LL_ADC_CHANNEL_6, LL_ADC_CHANNEL_7};

const uint32_t GPIO_PINS[] = {LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3,
                              LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7};

uint32_t counter;
uint8_t numSensor;  // number of active sensors
int activeSensors[MAX_SENSORS]; // which sensors are active
int activeSensorPairs[MAX_SENSORS/2];
uint32_t dmaBuffer[MAX_SENSORS/2];  // DMA reads both ADCs at the same time to one 32b value
uint8_t serialData[(MAX_SENSORS + 1) * 2];  // 16b per sensor and 16b for timestamp
bool sendData = false;
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
  LL_ADC_DeInit(ADC2);
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
  LL_ADC_DeInit(ADC1);
  LL_ADC_DeInit(ADC2);
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

  ADCCommonConfig.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV8;
  ADCCommonConfig.Multimode = LL_ADC_MULTI_DUAL_REG_SIMULT;  // regular simultaneous mode
  ADCCommonConfig.MultiDMATransfer = LL_ADC_MULTI_REG_DMA_UNLMT_2; // allow unlimited DMA transfers. MODE2 = half-words by ADC pairs
  ADCCommonConfig.MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSMP_DELAY_5CYCLES; // fastest possible mode

  // Apply settings
  if (LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADCCommonConfig) != SUCCESS) {
    Blink(1);
    exit(1);
  }
}

void configureADC(ADC_TypeDef* adc) {
  LL_ADC_InitTypeDef ADCConfig;
  LL_ADC_StructInit(&ADCConfig);

  ADCConfig.Resolution = LL_ADC_RESOLUTION_10B; // 10-bit ADC resolution
  ADCConfig.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT; // right-align data within 16b register
  ADCConfig.SequencersScanMode = ADC_SCANMODES[numSensor/2 - 1] == LL_ADC_REG_SEQ_SCAN_DISABLE ? LL_ADC_SEQ_SCAN_DISABLE: LL_ADC_SEQ_SCAN_ENABLE;  // enable scan only if there is more than one rank to convert

  if (LL_ADC_Init(adc, &ADCConfig) != SUCCESS) {
    Blink(2);
    exit(1);
  }
}

void configureADCChannels(ADC_TypeDef* adc, const bool master) {
  LL_ADC_REG_InitTypeDef ADCChannelConfig;
  LL_ADC_REG_StructInit(&ADCChannelConfig);

  ADCChannelConfig.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;   // trigger conversion from software
  ADCChannelConfig.SequencerLength = ADC_SCANMODES[numSensor/2 - 1];  // number of ranks to convert
  ADCChannelConfig.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE; // not using discontinous mode
  ADCChannelConfig.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS; // enable continuous conversion mode
  ADCChannelConfig.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED; // Allow unlimited transfers to DMA

  if (LL_ADC_REG_Init(adc, &ADCChannelConfig) != SUCCESS) {
    Blink(3);
    exit(1);
  }

  // Set which channels will be converted and in which order, as well as their sampling time
  // ADC1 does even sensors, ADC2 odd sensors
  for (uint8_t i = !master; i < numSensor; i+=2) {
    uint8_t sensor_id = activeSensors[i];
    LL_ADC_REG_SetSequencerRanks(adc, ADC_RANKS[i/2], ADC_CHANNELS[sensor_id]);
    LL_ADC_SetChannelSamplingTime(adc, ADC_CHANNELS[sensor_id], LL_ADC_SAMPLINGTIME_144CYCLES);
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

  DMAConfig.PeriphOrM2MSrcAddress =  LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA_MULTI);  // macro handles obtaining common data register for multi-ADC mode
  DMAConfig.MemoryOrM2MDstAddress = (uint32_t) &dmaBuffer[0]; // target is the buffer in RAM
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

  // enable interrupt on transfer-complete
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);

  // enable the DMA stream
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
}

void configureNVIC() {
  // set the DMA interrupt prio to be higher than USB to ensure constant sampling
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_GetPriority(OTG_FS_IRQn) - 1);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

extern "C" void DMA2_Stream0_IRQHandler() {
  // for timestamp packet, we use the sensor id 0b111 and set the marker bit
  // the host can recognize that this is not a sensor value because the marker bit
  // can only be set for sensor 0
  // so the timestamp packets are
  // 1 111 TTTT, where T are the upper 4 bits of the timestamp
  // 0 1 TTTTTT, where T are the lower 4 bits of the timestamp
  uint16_t t = micros();
  serialData[0] = (0b1111 << 4) | ((t & 0x3C0) >> 6);
  serialData[1] = ((0b01) << 6) | (t & 0x3F);
  
  for (uint8_t i = 0; i < numSensor; i++) {
    uint8_t sensor_id = activeSensors[i];
    // even sensors are current values from ADC1
    // odd sensors are voltage values from ADC2
    uint16_t level;
    if (i % 2 == 0) {
      level = __LL_ADC_MULTI_CONV_DATA_MASTER_SLAVE(LL_ADC_MULTI_MASTER, dmaBuffer[i/2]);
    } else {
      level = __LL_ADC_MULTI_CONV_DATA_MASTER_SLAVE(LL_ADC_MULTI_SLAVE, dmaBuffer[i/2]);
    }
#ifdef USE_DISPLAY
    if (displayEnabled) {
      // store in sensorValues for display purposes
      sensorLevels[sensor_id] = level;
    }
#endif
    // add metadata to remaining bits: 2 bytes available with 10b sensor value
    // First byte: 1 iii aaaa
    // where iii is the sensor id, a are the upper 4 bits of the level
    serialData[2*i + 2] = ((sensor_id & 0x7) << 4) | ((level & 0x3C0) >> 6) | (1 << 7);
    // Second byte: 0 m bbbbbb
    // where m is the marker bit, b are the lower 6 bits of the level
    serialData[2*i + 3] = ((sendMarkerNext << 6) | (level & 0x3F)) & ~(1 << 7);
    counter++;
    sendMarkerNext = false;
  }

  // trigger sending data to host if enabled
  if (streamValues | sendSingleValue) {
    sendData = true;
    sendSingleValue = false;
  }

  // clear DMA TC flag
  LL_DMA_ClearFlag_TC0(DMA2);
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
      break;
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
  LL_ADC_Disable(ADC2);
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);

  getActiveSensors();
  configureGPIO();
  configureDMA();
  configureADCCommon();
  configureADC(ADC1);
  configureADC(ADC2);
  configureADCChannels(ADC1, /* master */ true);
  configureADCChannels(ADC2, /* master */ false);
  configureNVIC();
  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);
  LL_ADC_REG_StartConversionSWStart(ADC1);  // ADC1 controls others ADCs in multi mode
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
  digitalWrite(LED_BUILTIN, LOW);

  // read virtual EEPROM data
  readEEPROMFromFlash();

  // enable clocks
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

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
  if (sendData) {
    Serial.write(serialData, 2 * (numSensor+1));
    sendData = false;
  }
  serialEvent();
  // update display if enabled
#ifdef USE_DISPLAY
  if (displayEnabled) {
    updateDisplay();
  }
#endif
}
