#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>


#define TFT_SCLK PB13
#define TFT_MOSI PB15
#define TFT_DC PA8
#define TFT_RST PA9
#define TFT_CS PA10
#define BACKLIGHT_PIN PB3

#define MAX_WIDTH  160
#define MAX_HEIGHT 80
#define OFFSET 24  // vertical offset: pixel 0 is outside of the screen


// Define class for SPI2 (as default SPI1 pins overlap with ADC)
// MOSI, MISO, SCLK
SPIClass SPI_2(PB15, PB14, PB13);

Adafruit_ST7735 tft = Adafruit_ST7735(&SPI_2, TFT_CS, TFT_DC, TFT_RST);

void initDisplay() {
  tft.initR(INITR_GREENTAB);
  tft.setRotation(3);
  tft.invertDisplay(true);
  tft.fillScreen(ST77XX_BLACK);
}

void displaySensor(int sensor, int totalWatt, float volt, float amp, float watt) {
  char buf[12];
  
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, OFFSET + 10);
  tft.setTextSize(5);
  tft.setTextColor(ST77XX_YELLOW);
  sprintf(buf, "%3d W", totalWatt);
  tft.print(buf);

  tft.setCursor(0, OFFSET + 65);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_BLUE);
  sprintf(buf, "S%1d:  ", sensor);
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


void updateDisplay() {
  static float volt = 3.3;
  static float amp = 2.0;
  static float watt = 6.6;
  static int totalWatt = watt * 10;
  static int sensor = 0;

  static unsigned long previousMillis = 0;
  unsigned long interval = (unsigned long)(millis() - previousMillis);
  if (interval > 2000) { 
    displaySensor(sensor, totalWatt, volt * (sensor + 1), amp * (sensor + 1), watt * (sensor + 1));
    sensor = (sensor + 1) % 4;
    previousMillis = millis();
  }
}
