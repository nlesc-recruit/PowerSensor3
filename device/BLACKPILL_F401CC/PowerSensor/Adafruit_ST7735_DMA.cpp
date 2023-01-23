#include "Adafruit_ST7735_DMA.h"

void Adafruit_ST7735_DMA::fillScreen(uint16_t color) {
  uint16_t swapped_color = __builtin_bswap16(color);

  size_t len = _width * _height;
  uint16_t buf[len];
  for (size_t i=0; i<len; i++) {
    buf[i] = swapped_color;
  }

  startWrite();
  setAddrWindow(0, 0, _width, _height);
  _spi->DMAtransfer(buf, sizeof(buf));
  endWrite();
}

void Adafruit_ST7735_DMA::writeColor(uint16_t color, uint32_t len) {
  if (!len) {
    return;
  }

  uint16_t swapped_color = __builtin_bswap16(color);
  uint16_t buf[len];
  for (size_t i=0; i <len; i++) {
    buf[i] = swapped_color;
  }
  _spi->DMAtransfer(buf, sizeof(buf));
}
