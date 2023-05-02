#ifdef STM32F401xC
#define TFT_SCLK PB3
#define TFT_MISO PB4  // unused, but needs to be defined in SPI setup
#define TFT_MOSI PB5
#define TFT_RST PB14
#define TFT_DC PB15
#define TFT_CS PB8
#define TFT_BLK PB9
#elif defined STM32F407xx
#define TFT_SCLK PB13
#define TFT_MISO PB14  // unused, but needs to be defined in SPI setup
#define TFT_MOSI PB15
#define TFT_RST PB11
#define TFT_DC PB12
#define TFT_CS PB8
#define TFT_BLK PB9
#endif

#define MAX_WIDTH  160
#define MAX_HEIGHT 80
#define OFFSET 24  // vertical offset: pixel 0 is outside of the screen
#define OFFSET_MAIN (OFFSET + 10)
#define OFFSET_BOTTOM (OFFSET + 65)

#include <STM32F4_SPI_DMA.h>

#include "Adafruit_ST7735_DMA.hpp"
#include "display.hpp"

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
  clearDisplay();
  analogWrite(TFT_BLK, 120);
  displayInitialValues();
}
void deinitDisplay() {
  clearDisplay();
  analogWrite(TFT_BLK, 0);
  SPI_3.end();
}

void clearDisplay() {
  tft.fillScreen(ST77XX_BLACK);
}

void displayInitialValues() {
  clearDisplay();

  tft.setCursor(0, OFFSET_MAIN);
  tft.setTextSize(5);

  tft.setTextColor(ST77XX_YELLOW);
  tft.print("  0 W");

  tft.setCursor(0, OFFSET_BOTTOM);
  tft.setTextSize(1);

  tft.setTextColor(ST77XX_BLUE);
  tft.print("S0:  ");
  tft.setTextColor(ST77XX_RED);
  tft.print(" 0.0V  ");
  tft.setTextColor(ST77XX_GREEN);
  tft.print(" 0.0A  ");
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("  0.0W");
}

void displaySensor(const int sensorPairName, const float amp, const float volt,
                   const float watt, const float totalWatt) {
  char buf[6];  // at most 5 characters plus terminator

  tft.setCursor(0, OFFSET_MAIN);
  tft.setTextSize(5);

  tft.setTextColor(ST77XX_YELLOW);
  snprintf(buf, sizeof(buf), "%3d", static_cast<int>(totalWatt));
  tft.drawFastNumber(buf, 3);

  /* the bottom row contains the sensor values. We need to find the right cursor location for each number
   * the layouw is S<s>:  <vv.v>V  <aa.a>A  <www.w>W
   * The first character is an S, followed by the sensor ID
   * multiply by (font width plus one for space) to get the cursor positions
   */
  tft.setTextSize(1);
  unsigned int cursorPos = FONT_NCOL + 1;  // start after the first S character
  tft.setCursor(cursorPos, OFFSET_BOTTOM);
  tft.setTextColor(ST77XX_BLUE);
  snprintf(buf, sizeof(buf), "%1d", sensorPairName);
  tft.drawFastNumber(buf, 1);

  cursorPos += 4 * (FONT_NCOL + 1);  // skip sensor value (1 character), :, two spaces
  tft.setCursor(cursorPos, OFFSET_BOTTOM);
  tft.setTextColor(ST77XX_RED);
  dtostrf(volt, 4, 1, buf);
  tft.drawFastNumber(buf, 4);

  cursorPos += 7 * (FONT_NCOL + 1);  // skip voltage (4 characters), V, 2 spaces
  tft.setCursor(cursorPos, OFFSET_BOTTOM);
  tft.setTextColor(ST77XX_GREEN);
  dtostrf(amp, 4, 1, buf);
  tft.drawFastNumber(buf, 4);

  cursorPos += 7 * (FONT_NCOL + 1);  // skip current (4 characters), A, 2 spaces
  tft.setCursor(cursorPos, OFFSET_BOTTOM);
  tft.setTextColor(ST77XX_YELLOW);
  dtostrf(watt, 5, 1, buf);
  tft.drawFastNumber(buf, 5);
}
