#define TFT_SCLK PB13
#define TFT_MISO PB14  // unused, but needs to be defined in SPI setup
#define TFT_MOSI PB15
#define TFT_RST PB11
#define TFT_DC PB12
#define TFT_CS PB8
#define TFT_BLK PB9

#define MAX_WIDTH  160
#define MAX_HEIGHT 80
#define OFFSET 24  // vertical offset: pixel 0 is outside of the screen

#include <STM32F4_SPI_DMA.h>
#include "Adafruit_ST7735_DMA.h"

#include <display.h>

// Define class for SPI3 (as default SPI1 pins overlap with ADC)
SPIDMAClass SPI_3(TFT_MOSI, TFT_MISO, TFT_SCLK);
Adafruit_ST7735_DMA tft = Adafruit_ST7735_DMA(&SPI_3, TFT_CS, TFT_DC, TFT_RST);

void initDisplay() {
  SPI_3.begin();
  tft.initR(INITR_GREENTAB);
  tft.setRotation(3);
#ifdef TFT_BLUE
  tft.invertDisplay(false);
#else
  tft.invertDisplay(true);
#endif
  tft.fillScreen(ST77XX_BLACK);
  analogWrite(TFT_BLK, 120);
}
void deinitDisplay() {
  tft.fillScreen(ST77XX_BLACK);
  analogWrite(TFT_BLK, 0);
  SPI_3.end();
}

void clearDisplay() {
  tft.fillScreen(ST77XX_BLACK);
}

void displaySensor(const int sensorPairName, const float amp, const float volt,
                   const float watt, const float totalWatt) {
  char buf[12];

  clearDisplay();

  tft.setCursor(0, OFFSET + 10);
  tft.setTextSize(5);
  tft.setTextColor(ST77XX_YELLOW);
  snprintf(buf, sizeof(buf), "%3d W", static_cast<int>(totalWatt));
  tft.print(buf);

  tft.setCursor(0, OFFSET + 65);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_BLUE);
  snprintf(buf, sizeof(buf), "S%1d:  ", sensorPairName);
  tft.print(buf);

  tft.setTextColor(ST77XX_RED);
  dtostrf(volt, 4, 1, buf);
  tft.print(strcat(buf, "V "));

  tft.setTextColor(ST77XX_GREEN);
  dtostrf(amp, 4, 1, buf);
  tft.print(strcat(buf, "A  "));

  tft.setTextColor(ST77XX_YELLOW);
  dtostrf(watt, 5, 1, buf);
  tft.print(strcat(buf, "W"));
}
