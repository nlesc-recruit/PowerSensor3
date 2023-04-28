#include "Adafruit_ST7735_DMA.h"
#include "fontMaps.cpp"

extern void Blink(uint8_t);

Adafruit_ST7735_DMA::Adafruit_ST7735_DMA(SPIDMAClass *spiClass, int8_t cs, int8_t dc, int8_t rst) :
  Adafruit_ST7735(static_cast<SPIClass*>(spiClass), cs, dc, rst),
  _spi(spiClass) {
    spiClass->begin();
  }

void Adafruit_ST7735_DMA::fillScreen(uint16_t color) {
  uint16_t swapped_color = __builtin_bswap16(color);

  size_t len = _width * _height;
  uint16_t* buf = new uint16_t[len];
  for (size_t i = 0; i < len; i++) {
    buf[i] = swapped_color;
  }

  startWrite();
  setAddrWindow(0, 0, _width, _height);
  _spi->DMAtransfer(buf, len * sizeof(uint16_t));
  endWrite();
  delete[] buf;
}

void Adafruit_ST7735_DMA::writeColor(uint16_t color, uint32_t len) {
  if (!len) {
    return;
  }

  uint16_t swapped_color = __builtin_bswap16(color);
  uint16_t* buf = new uint16_t[len];
  for (size_t i=0; i < len; i++) {
    buf[i] = swapped_color;
  }
  _spi->DMAtransfer(buf, len * sizeof(uint16_t));
  delete[] buf;
}

// drawChar from Adafruit GFX for default font
/*!
   @brief   Draw a single character
    @param    x   Bottom left corner x coordinate
    @param    y   Bottom left corner y coordinate
    @param    c   The 8-bit font-indexed character (likely ascii)
    @param    color 16-bit 5-6-5 Color to draw chraracter with
*/
void Adafruit_ST7735_DMA::drawFastChar(int16_t x, int16_t y, unsigned char c,
              uint16_t color) {
  static uint16_t valuesPerChar = 5*8;
  static uint16_t nChar = 11;

  // ascii has period, slash, then 0-9
  // check that we have a valid character to display and compute index 
  // in 0123456789.
  if (c > '9' | c < '0' | c == '/') {
    // invalid character, only digits and period supported
    // could fall back to pixel-by-pixel writing here
    return;
  }
  int idx = c - '0';
  if (c < 0) {
    // period, which is the eleventh character
    idx = 10;
  }

  // set pointer to start of the character in the correct fontmap
  uint16_t* charStart;
  switch (color) {
    case ST77XX_RED:
      charStart = const_cast<uint16_t*>(&fontMap[(nChar*0 + idx) * valuesPerChar]);
      break;
    case ST77XX_GREEN:
      charStart = const_cast<uint16_t*>(&fontMap[(nChar*1 + idx) * valuesPerChar]);
      break;
    case ST77XX_BLUE:
      charStart = const_cast<uint16_t*>(&fontMap[(nChar*2 + idx) * valuesPerChar]);
      break;
    case ST77XX_YELLOW:
      charStart = const_cast<uint16_t*>(&fontMap[(nChar*3 + idx) * valuesPerChar]);
      break;
  }

  startWrite();
  setAddrWindow(x, y, 5, 8);

  _spi->DMAtransfer(charStart, valuesPerChar * sizeof(uint16_t));
  endWrite();
}
