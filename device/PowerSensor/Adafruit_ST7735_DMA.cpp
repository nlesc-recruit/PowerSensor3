#include "Adafruit_ST7735_DMA.hpp"

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

void Adafruit_ST7735_DMA::writeFastChar(int16_t x, int16_t y, unsigned char c,
              uint16_t color, uint8_t size) {
  const uint16_t valuesPerChar = FONT_NROW * FONT_NCOL * size * size;

  // if the character is a space, we can simply write zeroes
  if (c == ' ') {
    setAddrWindow(x, y, FONT_NCOL * size, FONT_NROW * size);
    _spi->DMAtransfer(const_cast<uint16_t*>(&fontMapSpace[0]), valuesPerChar * sizeof(uint16_t));
    return;
  }

  // valid characters are ./0123456789
  // this is a continuous range in ascii, check if we have a valid character
  if (c < '.' | c > '9') {
    // could fall back to pixel-by-pixel writing here
    return;
  }
  // get index starting from period
  int idx = c - '.';
  // pointer to start of this character in the font map
  uint16_t* charStart;
  switch (size) {
    case (1):
      charStart = const_cast<uint16_t*>(&fontMap[colorMap.at(color) + idx * valuesPerChar]);
      break;
    case (5):
      // only yellow supported
      if (color != ST77XX_YELLOW) {
        return;
      }
      charStart = const_cast<uint16_t*>(&fontMapLarge[idx * valuesPerChar]);
      break;
    default:
      // other font sizes not supported
      return;
  }

  setAddrWindow(x, y, FONT_NCOL * size, FONT_NROW * size);
  _spi->DMAtransfer(charStart, valuesPerChar * sizeof(uint16_t));
}

void Adafruit_ST7735_DMA::drawFastNumber(char* number, uint16_t len) {
  startWrite();
  for (uint16_t i=0; i < len; i++) {
    writeFastChar(cursor_x, cursor_y, number[i], textcolor, textsize_x);  // assume text size x and y are equal
    cursor_x += (FONT_NCOL + 1) * textsize_x;
  }
  endWrite();
}
