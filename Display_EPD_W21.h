#ifndef DISPLAY_EPD_W21_H
#define DISPLAY_EPD_W21_H

#include <Arduino.h>

static constexpr uint8_t EPD_COLOR_BLACK = 0x00;
static constexpr uint8_t EPD_COLOR_WHITE = 0x01;
static constexpr uint8_t EPD_COLOR_YELLOW = 0x02;
static constexpr uint8_t EPD_COLOR_RED = 0x03;

static constexpr uint16_t EPD_WIDTH = 152;
static constexpr uint16_t EPD_HEIGHT = 152;
static constexpr uint16_t EPD_FRAME_BYTES = EPD_WIDTH * EPD_HEIGHT / 4;

void EPD_init(void);
void EPD_sleep(void);
void EPD_refresh(void);
bool EPD_waitReady(const char *phase, uint32_t timeoutMs = 60000);
void EPD_displayNative(const uint8_t *frame);

#endif
