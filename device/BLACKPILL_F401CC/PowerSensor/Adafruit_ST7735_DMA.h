#ifndef ADAFRUIT_ST7735_DMA_H
#define ADAFRUIT_ST7735_DMA_H

#include <Adafruit_SPITFT.h>
#include <Adafruit_ST7735.h>
#include <STM32F4_SPI_DMA.h>

class Adafruit_ST7735_DMA: public Adafruit_ST7735 {
  public:
    Adafruit_ST7735_DMA(SPIDMAClass *spiClass, int8_t cs, int8_t dc, int8_t rst) : Adafruit_ST7735(static_cast<SPIClass*>(spiClass), cs, dc, rst), _spi(spiClass) {
    spiClass->begin();
    }

    void fillScreen(uint16_t color);
    void writeColor(uint16_t color, uint32_t len) override;

  private:
    SPIDMAClass* _spi;
};

#endif  // ADAFRUIT_ST7735_DMA_H
