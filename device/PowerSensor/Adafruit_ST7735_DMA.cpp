#include "Adafruit_ST7735_DMA.h"

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
