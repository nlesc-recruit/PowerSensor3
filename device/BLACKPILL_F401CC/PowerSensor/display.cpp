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


#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735
#include <SPI.h>

#include <display.h>


// Define class for SPI3 (as default SPI1 pins overlap with ADC)
SPIClass SPI_2(TFT_MOSI, TFT_MISO, TFT_SCLK);

Adafruit_ST7735 tft = Adafruit_ST7735(&SPI_2, TFT_CS, TFT_DC, TFT_RST);

void initDisplay() {
  tft.initR(INITR_GREENTAB);
  tft.setRotation(3);
  tft.invertDisplay(true);
  tft.fillScreen(ST77XX_BLACK);
  analogWrite(TFT_BLK, 120);
}

void displaySensor(const int sensorPairName, const float amp, const float volt,
                   const float watt, const float totalWatt) {
  char buf[12];

  tft.fillScreen(ST77XX_BLACK);
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
