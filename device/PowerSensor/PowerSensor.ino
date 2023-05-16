/* NOTE: to avoid dropping data one must increase the USB transmit buffer size
 * See CDC_TRANSMIT_QUEUE_BUFFER_SIZE in Arduino15/packages/STMicroelectronics/hardware/stm32/2.3.0/cores/arduino/stm32/usb/cdc/cdc_queue.h
 * A value of 6 times CDC_TRANSMIT_QUEUE_BUFFER_SIZE instead of the default 2 times CDC_TRANSMIT_QUEUE_BUFFER_SIZE seems to be enough
 */

#define USE_FULL_LL_DRIVER
#define SENSORS 8  // limited by number of bits used for sensor id
#define PAIRS 4

#ifdef STM32F401xC
#define VERSION "F401-0.1.0"
#elif defined STM32F407xx
#define VERSION "F407-0.1.0"
#else
#error "Unsupported device"
#endif

/* The following two values are used to be able to jump to the bootloader from the application
 * Start of system memory is 0x1FFF 0000, see Table 3 in the uC reference manual
 * at boot the first word contains the initial value of the stack pointer.
 * To be safe we put the stack at the end of the first RAM block.
 * RAM starts at 0x2000 0000 and is either 12 KB (F401CC) or 64 KB (F407VG) in size
 * the second word stores the address at which code execution should start,
 * this is where we jump to to run the bootloader
 */
#define SYSMEM_RESET_VECTOR            0x1FFF0004
#ifdef STM32F401xC
#define BOOTLOADER_STACK_POINTER       0x2000FFFF
#elif defined STM32F407xx
#define BOOTLOADER_STACK_POINTER       0x2001BFFF
#endif

#include <stm32f4xx_ll_bus.h>   // clock control
#include <stm32f4xx_ll_adc.h>   // ADC control
#include <stm32f4xx_ll_gpio.h>  // GPIO control
#include <stm32f4xx_ll_dma.h>   // DMA control
#include <EEPROM.h>

#ifndef NODISPLAY
#define UPDATE_INVERVAL 2000  // ms
#define VOLTAGE 3.3
#define MAX_LEVEL 1023
#include "display.hpp"
uint16_t sensorLevels[SENSORS];  // to store sensor values for displaying purposes
float voltageValues[PAIRS];
float currentValues[PAIRS];
float powerValues[PAIRS];
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

#ifdef STM32F401xC
const int numSampleToAverage = 6;  // number of samples to average
__IO uint16_t dmaBuffer[SENSORS];  // 16b per sensor
uint16_t avgBuffer[SENSORS][numSampleToAverage];
uint16_t currentSample = 0;
#elif defined STM32F407xx
__IO uint32_t dmaBuffer[PAIRS];  // DMA reads both ADCs at the same time to one 32b value
#endif
uint32_t counter = 0;
uint8_t serialData[(SENSORS + 1) * 2];  // 16b per sensor and 16b for timestamp
bool sendData = false;
bool streamValues = false;
bool sendSingleValue = false;
bool sendMarkerNext = false;

// include device-specific code for setting up the ADC and DMA
#ifdef STM32F401xC
#include "device_specific/BLACKPILL_F401CC.hpp"
#elif defined STM32F407xx
#include "device_specific/DISCO_F407VG.hpp"
#endif

struct Sensor {
  char type[16];
  float vref;
  float sensitivity;
  bool inUse;
} __attribute__((packed));

struct Config {
  Sensor sensors[SENSORS];
} eeprom;

void JumpToBootloader() {
  // ensure DMA and ADC are off
  LL_ADC_Disable(ADC1);
#ifdef STM32F407xx
  LL_ADC_Disable(ADC2);
#endif
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
  LL_ADC_DeInit(ADC1);
#ifdef STM32F407xx
  LL_ADC_DeInit(ADC2);
#endif
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_0);

  // function starting at to whatever address is stored in the reset vector
  void (*SysMemBootJump)(void) = (void (*)(void)) (*(reinterpret_cast<uint32_t *>(SYSMEM_RESET_VECTOR)));
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
  for (int s=0; s < SENSORS; s++) {
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
  uint8_t* p_eeprom = reinterpret_cast<uint8_t*>(&eeprom);
  for (int s=0; s < SENSORS; s++) {
    // wait for entire sensor chunk to be available
    while (Serial.available() < sizeof(Sensor)) {
    }
    // write sensor bytes into eeprom struct
    for (int b=0; b < sizeof(Sensor); b++) {
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
  uint8_t* p_eeprom = reinterpret_cast<uint8_t*>(&eeprom);
  for (uint16_t i=0; i < sizeof eeprom; i++) {
    *p_eeprom++ = eeprom_buffered_read_byte(i);
  }
}

void writeEEPROMToFlash() {
  // write buffer per byte
  uint8_t* p_eeprom = reinterpret_cast<uint8_t*>(&eeprom);
  for (uint16_t i=0; i < sizeof eeprom; i++) {
    eeprom_buffered_write_byte(i, *p_eeprom++);
  }
  // copy from buffer to flash
  eeprom_buffer_flush();
}

void configureGPIO() {
  // Create GPIO config struct and fill with defaults
  LL_GPIO_InitTypeDef GPIOConfig;
  LL_GPIO_StructInit(&GPIOConfig);

  uint32_t pins = 0;
  for (uint8_t i = 0; i < SENSORS; i++) {
    pins |= GPIO_PINS[i];
  }

  GPIOConfig.Pin = pins;
  GPIOConfig.Mode = LL_GPIO_MODE_ANALOG;

  if (LL_GPIO_Init(GPIOA, &GPIOConfig) != SUCCESS) {
    Blink(4);
    exit(1);
  }
}

void configureNVIC() {
  // set the DMA interrupt prio to be higher than USB to ensure constant sampling
  // NOTE: this means USB communication cannot be used in the DMA interrupt handler
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_GetPriority(OTG_FS_IRQn) - 1);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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
      Serial.write((const uint8_t[]) { 0xFF, 0x3F}, 2);
      Serial.write((const uint8_t[]) { 0xFF, 0x3F}, 2);
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
#ifndef NODISPLAY
    case 'D':
      // toggle display
      displayEnabled = !displayEnabled;
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

  configureGPIO();
  configureDMA();
  configureADCCommon();
#ifdef STM32F401xC
  configureADC();
  configureADCChannels();
#elif defined STM32F407xx
  configureADC(ADC1);
  configureADC(ADC2);
  configureADCChannels(ADC1, /* master */ true);
  configureADCChannels(ADC2, /* master */ false);
#endif
  configureNVIC();
  LL_ADC_Enable(ADC1);
#ifdef STM32F407xx
  LL_ADC_Enable(ADC2);
#endif
  LL_ADC_REG_StartConversionSWStart(ADC1);
}


#ifndef NODISPLAY
void updateCalibratedSensorValues() {
  totalPower = 0;
  for (int pair=0; pair < PAIRS; pair++) {
    if (eeprom.sensors[2 * pair].inUse & eeprom.sensors[2 * pair + 1].inUse) {
      float amp = (VOLTAGE * sensorLevels[2 * pair] / MAX_LEVEL
        - eeprom.sensors[2 * pair].vref) / eeprom.sensors[2 * pair].sensitivity;
      float volt = (VOLTAGE * sensorLevels[2 * pair + 1] / MAX_LEVEL
        - eeprom.sensors[2 * pair + 1].vref) / eeprom.sensors[2 * pair + 1].sensitivity;
      float power = volt * amp;
      voltageValues[pair] = volt;
      currentValues[pair] = amp;
      powerValues[pair] = power;
      totalPower += power;
    }
  }
}

void updateDisplay() {
  static unsigned long previousMillis = 0;
  unsigned long interval = (unsigned long)(millis() - previousMillis);

  if (interval > UPDATE_INVERVAL) {
    static unsigned int sensor_pair = 0;

    previousMillis = millis();
    // update the values, then write to display
    sensor_pair = (sensor_pair + 1) % PAIRS;
    updateCalibratedSensorValues();
    if (eeprom.sensors[2 * sensor_pair].inUse & eeprom.sensors[2 * sensor_pair + 1].inUse) {
      displaySensor(sensor_pair, currentValues[sensor_pair],
        voltageValues[sensor_pair], powerValues[sensor_pair], totalPower);
    }
  }
}
#endif


void setup() {
  Serial.begin();
  pinMode(LED_BUILTIN, OUTPUT);
#ifdef STM32F401xC
  digitalWrite(LED_BUILTIN, LOW);
#elif defined STM32F407xx
  digitalWrite(LED_BUILTIN, HIGH);  // HIGH is off on F407
#endif

  // read virtual EEPROM data
  readEEPROMFromFlash();

  // enable clocks
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
#ifdef STM32F407xx
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
#endif

  // configure hardware (GPIO, DMA, ADC)
  configureDevice();
#ifndef NODISPLAY
  if (displayEnabled) {
    initDisplay();
  }
#endif
  Blink(1);
}

void loop() {
  if (sendData) {
    Serial.write(serialData, sizeof(serialData));
    sendData = false;
  }
  // only check for serial events, sending sensor values to host is handled through interrupts
  serialEvent();
  // update display if enabled
#ifndef NODISPLAY
  if (displayEnabled) {
    updateDisplay();
  }
#endif
}
