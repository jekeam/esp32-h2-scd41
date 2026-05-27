#include "Display_EPD_W21_spi.h"

#include <SPI.h>

static SPISettings epdSpiSettings(4000000, MSBFIRST, SPI_MODE0);

void EPD_ioInit(void) {
  pinMode(EPD_BUSY_PIN, INPUT_PULLUP);
  pinMode(EPD_RST_PIN, OUTPUT);
  pinMode(EPD_DC_PIN, OUTPUT);
  pinMode(EPD_CS_PIN, OUTPUT);

  digitalWrite(EPD_CS_PIN, HIGH);
  digitalWrite(EPD_DC_PIN, HIGH);
  digitalWrite(EPD_RST_PIN, HIGH);

  SPI.begin(EPD_SCK_PIN, -1, EPD_MOSI_PIN, EPD_CS_PIN);
}

void EPD_W21_WriteCMD(uint8_t command) {
  SPI.beginTransaction(epdSpiSettings);
  digitalWrite(EPD_CS_PIN, LOW);
  digitalWrite(EPD_DC_PIN, LOW);
  SPI.transfer(command);
  digitalWrite(EPD_CS_PIN, HIGH);
  SPI.endTransaction();
}

void EPD_W21_WriteDATA(uint8_t data) {
  SPI.beginTransaction(epdSpiSettings);
  digitalWrite(EPD_CS_PIN, LOW);
  digitalWrite(EPD_DC_PIN, HIGH);
  SPI.transfer(data);
  digitalWrite(EPD_CS_PIN, HIGH);
  SPI.endTransaction();
}

void EPD_W21_WriteDATA_Bulk(const uint8_t *data, size_t len) {
  SPI.beginTransaction(epdSpiSettings);
  digitalWrite(EPD_CS_PIN, LOW);
  digitalWrite(EPD_DC_PIN, HIGH);
  for (size_t i = 0; i < len; ++i) {
    SPI.transfer(data[i]);
  }
  digitalWrite(EPD_CS_PIN, HIGH);
  SPI.endTransaction();
}
