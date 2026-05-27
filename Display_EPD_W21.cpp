#include "Display_EPD_W21.h"

#include "Display_EPD_W21_spi.h"

bool EPD_waitReady(const char *phase, uint32_t timeoutMs) {
  const uint32_t start = millis();
  const int startBusy = digitalRead(EPD_BUSY_PIN);
  Serial.printf("[EPD] BUSY wait begin: %s, pin=%d, timeout=%lu ms\n", phase, startBusy, (unsigned long)timeoutMs);
  Serial.flush();
  while (digitalRead(EPD_BUSY_PIN) == LOW) {
    delay(10);
    yield();
    if (millis() - start > timeoutMs) {
      Serial.printf("[EPD] BUSY wait TIMEOUT: %s, elapsed=%lu ms, pin=%d\n", phase, (unsigned long)(millis() - start), digitalRead(EPD_BUSY_PIN));
      Serial.flush();
      return false;
    }
  }
  Serial.printf("[EPD] BUSY wait done: %s, elapsed=%lu ms\n", phase, (unsigned long)(millis() - start));
  Serial.flush();
  return true;
}

void EPD_init(void) {
  const uint32_t start = millis();
  Serial.println("[EPD] init begin");
  Serial.flush();
  delay(20);
  digitalWrite(EPD_RST_PIN, LOW);
  delay(40);
  digitalWrite(EPD_RST_PIN, HIGH);
  delay(50);

  EPD_waitReady("after reset");

  Serial.println("[EPD] init commands begin");
  Serial.flush();
  EPD_W21_WriteCMD(0x4D);
  EPD_W21_WriteDATA(0x78);

  EPD_W21_WriteCMD(0x00);
  EPD_W21_WriteDATA(0x0F);
  EPD_W21_WriteDATA(0x29);

  EPD_W21_WriteCMD(0x01);
  EPD_W21_WriteDATA(0x07);
  EPD_W21_WriteDATA(0x00);

  EPD_W21_WriteCMD(0x03);
  EPD_W21_WriteDATA(0x10);
  EPD_W21_WriteDATA(0x54);
  EPD_W21_WriteDATA(0x44);

  EPD_W21_WriteCMD(0x06);
  EPD_W21_WriteDATA(0x05);
  EPD_W21_WriteDATA(0x00);
  EPD_W21_WriteDATA(0x3F);
  EPD_W21_WriteDATA(0x0A);
  EPD_W21_WriteDATA(0x25);
  EPD_W21_WriteDATA(0x12);
  EPD_W21_WriteDATA(0x1A);

  EPD_W21_WriteCMD(0x50);
  EPD_W21_WriteDATA(0x37);

  EPD_W21_WriteCMD(0x60);
  EPD_W21_WriteDATA(0x02);
  EPD_W21_WriteDATA(0x02);

  EPD_W21_WriteCMD(0x61);
  EPD_W21_WriteDATA(EPD_WIDTH / 256);
  EPD_W21_WriteDATA(EPD_WIDTH % 256);
  EPD_W21_WriteDATA(EPD_HEIGHT / 256);
  EPD_W21_WriteDATA(EPD_HEIGHT % 256);

  EPD_W21_WriteCMD(0xE7);
  EPD_W21_WriteDATA(0x1C);

  EPD_W21_WriteCMD(0xE3);
  EPD_W21_WriteDATA(0x22);

  EPD_W21_WriteCMD(0xB4);
  EPD_W21_WriteDATA(0xD0);
  EPD_W21_WriteCMD(0xB5);
  EPD_W21_WriteDATA(0x03);

  EPD_W21_WriteCMD(0xE9);
  EPD_W21_WriteDATA(0x01);

  EPD_W21_WriteCMD(0x30);
  EPD_W21_WriteDATA(0x08);

  EPD_W21_WriteCMD(0x04);
  EPD_waitReady("power on");
  Serial.printf("[EPD] init done, elapsed=%lu ms\n", (unsigned long)(millis() - start));
  Serial.flush();
}

void EPD_sleep(void) {
  const uint32_t start = millis();
  Serial.println("[EPD] sleep begin");
  Serial.flush();
  EPD_W21_WriteCMD(0x02);
  EPD_waitReady("power off");
  delay(100);

  EPD_W21_WriteCMD(0x07);
  EPD_W21_WriteDATA(0xA5);
  Serial.printf("[EPD] sleep done, elapsed=%lu ms\n", (unsigned long)(millis() - start));
  Serial.flush();
}

void EPD_refresh(void) {
  const uint32_t start = millis();
  Serial.println("[EPD] refresh command begin");
  Serial.flush();
  EPD_W21_WriteCMD(0x12);
  EPD_W21_WriteDATA(0x00);
  EPD_waitReady("display refresh");
  Serial.printf("[EPD] refresh command done, elapsed=%lu ms\n", (unsigned long)(millis() - start));
  Serial.flush();
}

void EPD_displayNative(const uint8_t *frame) {
  const uint32_t start = millis();
  Serial.printf("[EPD] frame write begin, bytes=%u\n", EPD_FRAME_BYTES);
  Serial.flush();
  EPD_W21_WriteCMD(0x10);
  EPD_W21_WriteDATA_Bulk(frame, EPD_FRAME_BYTES);
  Serial.printf("[EPD] frame write done, elapsed=%lu ms\n", (unsigned long)(millis() - start));
  Serial.flush();
  EPD_refresh();
}
