/* NOTE: to avoid dropping data one must increase the USB transmit buffer size
 * See CDC_TRANSMIT_QUEUE_BUFFER_SIZE in packages/STMicroelectronics/hardware/stm32/2.3.0/cores/arduino/stm32/usb/cdc/cdc_queue.h
 * A value of 6 times CDC_TRANSMIT_QUEUE_BUFFER_SIZE instead of the default 2 times CDC_TRANSMIT_QUEUE_BUFFER_SIZE seems to be enough
 */

#define USE_FULL_LL_DRIVER
#define SENSORS 8  // limited by number of bits used for sensor id
#define PAIRS 4
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
#define MAX_LEVEL 1023
#include "display.h"
uint16_t sensorLevels[SENSORS];  // to store raw (averaged) sensor values for displaying purposes
float voltageValues[PAIRS];
float currentValues[PAIRS];
float powerValues[PAIRS];
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
  Sensor sensors[SENSORS];
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
  for (int s=0; s<SENSORS; s++) {
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
  for (int s=0; s<SENSORS; s++) {
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
  for (int pair=0; pair < PAIRS; pair++) {
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
    static unsigned int sensor_pair = 0;
    previousMillis = millis();
    // update the values, then write to display
    sensor_pair = (sensor_pair + 1) % PAIRS;
    updateCalibratedSensorValues();
    displaySensor(sensor_pair, currentValues[sensor_pair], voltageValues[sensor_pair], powerValues[sensor_pair], totalPower);
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
    Serial.write(serialData, sizeof(serialData));
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
