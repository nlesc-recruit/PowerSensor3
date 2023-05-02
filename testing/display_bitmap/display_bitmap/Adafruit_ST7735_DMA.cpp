#include "Adafruit_ST7735_DMA.h"
#include "fontMaps.h"

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

  // valid characters are ./0123456789
  // this is a continuous range in ascii, check if we have a valid character
  if (c < '.' | c > '9') {
    // could fall back to pixel-by-pixel writing here
    return;
  }
  // get index starting from period
  int idx = c - '.';
  // pointer to start of this character in the font map
  uint16_t* charStart = const_cast<uint16_t*>(&fontMap[colorMap.at(color) + idx * valuesPerChar]);

  startWrite();
  setAddrWindow(x, y, 5, 8);

  _spi->DMAtransfer(charStart, valuesPerChar * sizeof(uint16_t));
  endWrite();
}