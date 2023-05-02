#ifndef ADAFRUIT_ST7735_DMA_H
#define ADAFRUIT_ST7735_DMA_H

#include <Adafruit_SPITFT.h>
#include <Adafruit_ST7735.h>
#include <STM32F4_SPI_DMA.h>
#include <fontMaps.h>

class Adafruit_ST7735_DMA: public Adafruit_ST7735 {
 public:
    Adafruit_ST7735_DMA(SPIDMAClass *spiClass, int8_t cs, int8_t dc, int8_t rst);

    void fillScreen(uint16_t color) override;
    void writeColor(uint16_t color, uint32_t len) override;
    void drawFastNumber(char* number, uint16_t len);

 private:
   void writeFastChar(int16_t x, int16_t y, unsigned char c,
                     uint16_t color, uint8_t size);
    SPIDMAClass* _spi;
    
};

#endif  // ADAFRUIT_ST7735_DMA_H
