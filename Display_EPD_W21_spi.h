#ifndef DISPLAY_EPD_W21_SPI_H
#define DISPLAY_EPD_W21_SPI_H

#include <Arduino.h>
#include <stddef.h>

// DESPI-C02 -> ESP32-H2 Super Mini wiring.
#ifndef EPD_BUSY_PIN
#define EPD_BUSY_PIN 0
#endif
#ifndef EPD_RST_PIN
#define EPD_RST_PIN 1
#endif
#ifndef EPD_DC_PIN
#define EPD_DC_PIN 2
#endif
#ifndef EPD_CS_PIN
#define EPD_CS_PIN 3
#endif
#ifndef EPD_SCK_PIN
#define EPD_SCK_PIN 4
#endif
#ifndef EPD_MOSI_PIN
#define EPD_MOSI_PIN 5
#endif

void EPD_ioInit(void);
void EPD_W21_WriteCMD(uint8_t command);
void EPD_W21_WriteDATA(uint8_t data);
void EPD_W21_WriteDATA_Bulk(const uint8_t *data, size_t len);

#endif
