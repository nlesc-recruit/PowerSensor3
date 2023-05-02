#define TFT_SCLK PB3
#define TFT_MISO PB4  // unused, but needs to be defined in SPI setup
#define TFT_MOSI PB5
#define TFT_RST PB14
#define TFT_DC PB15
#define TFT_CS PB8
#define TFT_BLK PB9

#define MAX_WIDTH  160
#define MAX_HEIGHT 80
#define OFFSET 24  // vertical offset: pixel 0 is outside of the screen

#include <STM32F4_SPI_DMA.h>
#include "Adafruit_ST7735_DMA.h"

SPIDMAClass SPI_3(TFT_MOSI, TFT_MISO, TFT_SCLK);
Adafruit_ST7735_DMA tft = Adafruit_ST7735_DMA(&SPI_3, TFT_CS, TFT_DC, TFT_RST);

void Blink(uint8_t amount) {
  // Blink LED, note that outputs are inverted: LOW is on, HIGH is off
  for (uint8_t i = 0; i < amount; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
  }
}


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    tft.initR(INITR_GREENTAB);
    tft.setRotation(3);
    tft.invertDisplay(true);
    tft.fillScreen(ST77XX_BLACK);
    analogWrite(TFT_BLK, 120);
}

void loop() {
    static char c = '0';
    tft.fillScreen(ST77XX_BLACK);

    uint8_t size = 1;
    tft.drawFastChar(MAX_WIDTH/2, OFFSET + 0, c, ST77XX_RED, size);
    tft.drawFastChar(MAX_WIDTH/2, OFFSET + 8, c, ST77XX_GREEN, size);
    tft.drawFastChar(MAX_WIDTH/2, OFFSET + 16, c, ST77XX_BLUE, size);
    tft.drawFastChar(MAX_WIDTH/2, OFFSET + 24, c, ST77XX_YELLOW, size);
    size = 5;
    tft.drawFastChar(MAX_WIDTH/2, OFFSET + 32, c, ST77XX_YELLOW, size);
    
    c++;
    if (c == '9' + 1) {
      // after 9, go to period
      c = '.';
    } else if (c == '.' + 1) {
      // after period, go to zero
      c = '0';
    }
    delay(250);
}
