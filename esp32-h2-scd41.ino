#ifndef ZIGBEE_MODE_ED
#error "Zigbee End Device mode is not selected (Tools -> Zigbee mode -> End Device)."
#endif

#include <Wire.h>
#include <Zigbee.h>
#include <Preferences.h>
#include <esp_zigbee_core.h>
#include <esp_err.h>
#include <esp_ieee802154.h>
#include <nwk/esp_zigbee_nwk.h>
#include <math.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <SensirionCore.h>
#include <SensirionI2cScd4x.h>

#include "Display_EPD_W21.h"
#include "Display_EPD_W21_spi.h"

// ---------- Pins ----------
#define I2C_SDA 10  // ESP32-H2 Super Mini
#define I2C_SCL 11  // ESP32-H2 Super Mini

#define WS2812_GPIO 8
#define WS2812_LEDS 1

// DESPI-C02 wiring is defined in Display_EPD_W21_spi.h:
// BUSY=GPIO0, RES=GPIO1, D/C=GPIO2, CS=GPIO3, SCK=GPIO4, SDI/MOSI=GPIO5.

// ---------- SCD4x ----------
static constexpr uint8_t SCD4X_I2C_ADDR = 0x62;

// ---------- Zigbee endpoints ----------
#define EP_TEMP_HUM 10
#define EP_CO2 11
#define EP_LED_DIM 12
#define EP_ALARM 13
#define EP_DISPLAY_REFRESH 14

// ---------- Timing ----------
static constexpr uint32_t SENSOR_POLL_MS = 1000;
static constexpr uint32_t ZB_REPORT_MS = 30000;
static constexpr uint16_t DISPLAY_REFRESH_INTERVAL_MIN = 1;
static constexpr uint16_t DISPLAY_REFRESH_INTERVAL_MAX = 60;
static constexpr uint16_t DISPLAY_REFRESH_INTERVAL_DEFAULT_MIN = 1;
static uint16_t display_refresh_interval_minutes = DISPLAY_REFRESH_INTERVAL_DEFAULT_MIN;
static constexpr uint32_t EPAPER_SKIP_LOG_MS = 30000;
static constexpr uint32_t ZIGBEE_LQI_POLL_MS = 10000;
static constexpr uint32_t ZIGBEE_READY_DELAY_MS = 2000;
static constexpr int8_t ZIGBEE_TX_POWER_DBM = 20;
static constexpr uint16_t EPAPER_CO2_DELTA_PPM = 50;
static constexpr float EPAPER_TEMP_DELTA_C = 0.5f;
static constexpr float EPAPER_RH_DELTA = 2.0f;

static constexpr bool LED_ENABLED_DEFAULT = true;
static constexpr uint8_t LED_ZIGBEE_LEVEL_DEFAULT = 255;

static uint32_t epaperRefreshIntervalMs() {
  const uint16_t minutes = display_refresh_interval_minutes
                             ? display_refresh_interval_minutes
                             : DISPLAY_REFRESH_INTERVAL_DEFAULT_MIN;
  return (uint32_t)minutes * 60UL * 1000UL;
}

static uint16_t clampDisplayRefreshMinutes(int32_t minutes) {
  if (minutes < DISPLAY_REFRESH_INTERVAL_MIN) return DISPLAY_REFRESH_INTERVAL_MIN;
  if (minutes > DISPLAY_REFRESH_INTERVAL_MAX) return DISPLAY_REFRESH_INTERVAL_MAX;
  return (uint16_t)minutes;
}

static void loadDisplayRefreshInterval() {
  Preferences prefs;
  if (!prefs.begin("co2meter", true)) {
    Serial.printf("[CFG] display refresh interval load failed, using default %u min\n",
                  DISPLAY_REFRESH_INTERVAL_DEFAULT_MIN);
    return;
  }

  const uint16_t saved = prefs.getUShort("refresh_min", DISPLAY_REFRESH_INTERVAL_DEFAULT_MIN);
  prefs.end();

  display_refresh_interval_minutes = clampDisplayRefreshMinutes(saved);
  Serial.printf("[CFG] display refresh interval loaded: %u min",
                display_refresh_interval_minutes);
  if (saved != display_refresh_interval_minutes) {
    Serial.printf(" (saved=%u clamped)", saved);
  }
  Serial.println();
}

static void saveDisplayRefreshInterval(uint16_t minutes) {
  Preferences prefs;
  if (!prefs.begin("co2meter", false)) {
    Serial.printf("[CFG] display refresh interval save failed: %u min\n", minutes);
    return;
  }

  const size_t written = prefs.putUShort("refresh_min", minutes);
  prefs.end();

  Serial.printf("[CFG] display refresh interval saved: %u min, result=%s\n",
                minutes,
                written ? "ok" : "FAIL");
}

static uint8_t ledZigbeeLevelToPercent(uint8_t level) {
  return (uint8_t)(((uint16_t)level * 100U + 127U) / 255U);
}

static void loadLedState(bool &enabled, uint8_t &zigbeeLevel) {
  Preferences prefs;
  enabled = LED_ENABLED_DEFAULT;
  zigbeeLevel = LED_ZIGBEE_LEVEL_DEFAULT;

  if (!prefs.begin("co2meter", true)) {
    Serial.printf("[CFG] LED state load failed, using default: on=%u, zb_level=%u\n",
                  enabled ? 1 : 0,
                  zigbeeLevel);
    return;
  }

  enabled = prefs.getBool("led_on", LED_ENABLED_DEFAULT);
  zigbeeLevel = prefs.getUChar("led_level", LED_ZIGBEE_LEVEL_DEFAULT);
  prefs.end();

  Serial.printf("[CFG] LED state loaded: on=%u, zb_level=%u, brightness=%u%%\n",
                enabled ? 1 : 0,
                zigbeeLevel,
                ledZigbeeLevelToPercent(zigbeeLevel));
}

static void saveLedState(bool enabled, uint8_t zigbeeLevel) {
  Preferences prefs;
  if (!prefs.begin("co2meter", false)) {
    Serial.printf("[CFG] LED state save failed: on=%u, zb_level=%u\n",
                  enabled ? 1 : 0,
                  zigbeeLevel);
    return;
  }

  const size_t writtenOn = prefs.putBool("led_on", enabled);
  const size_t writtenLevel = prefs.putUChar("led_level", zigbeeLevel);
  prefs.end();

  Serial.printf("[CFG] LED state saved: on=%u, zb_level=%u, brightness=%u%%, result=%s\n",
                enabled ? 1 : 0,
                zigbeeLevel,
                ledZigbeeLevelToPercent(zigbeeLevel),
                (writtenOn && writtenLevel) ? "ok" : "FAIL");
}

// ---------- PPM limits ----------
static uint16_t green_ppm = 800;
static uint16_t yellow_ppm = 1200;
static uint16_t orange_ppm = 1800;
static uint16_t red_ppm = 4999;

// ---------- Objects ----------
Adafruit_NeoPixel pixels(WS2812_LEDS, WS2812_GPIO, NEO_GRB + NEO_KHZ800);
SensirionI2cScd4x scd4x;

ZigbeeTempSensor zbTempHum(EP_TEMP_HUM);
ZigbeeCarbonDioxideSensor zbCO2(EP_CO2);
ZigbeeDimmableLight zbLedDim(EP_LED_DIM);
ZigbeeBinary zbAlarm(EP_ALARM);
ZigbeeAnalog zbDisplayRefresh(EP_DISPLAY_REFRESH);

// ---------- State ----------
static const uint8_t button = BOOT_PIN;

static bool g_zigbeeStarted = false;
static bool g_zigbeeReportingConfigured = false;
static bool g_zclReady = false;
static bool g_zhaSeen = false;
static bool g_zigbeeTxPowerConfigured = false;
static bool g_zigbeeTxPowerAppliedAfterJoin = false;
static bool g_lastZigbeeConnected = false;
static bool g_forceEpaperUpdate = false;
static bool g_displayRefreshAnalogSyncing = false;
static bool g_displayRefreshAnalogSyncPending = false;
static uint32_t g_identifyUntil = 0;
static uint32_t g_lastIdentifyBlink = 0;
static bool g_identifyBlinkOn = false;
static uint32_t g_connectedAt = 0;
static uint32_t g_lastZigbeeLqiPoll = 0;
static int16_t g_zigbeeLqi = -1;
static int8_t g_zigbeeRssi = 0;
static uint8_t g_zigbeeChannel = 0;
static uint16_t g_zigbeeShortAddr = 0xFFFF;
static uint16_t g_zigbeeParentShortAddr = 0xFFFF;

static bool g_ledEnabled = LED_ENABLED_DEFAULT;
static uint8_t g_ledZigbeeLevel = LED_ZIGBEE_LEVEL_DEFAULT;
static uint8_t g_ledLevel100 = 100;
static bool g_alarm = false;

static uint16_t g_lastCO2 = 0;
static float g_lastTempC = NAN;
static float g_lastRh = NAN;
static bool g_hasMeasurement = false;

static uint32_t lastPoll = 0;
static uint32_t lastReport = 0;
static uint32_t lastEpaper = 0;
static uint32_t lastEpaperSkipLog = 0;
static uint16_t lastEpaperCO2 = 0;
static float lastEpaperTempC = NAN;
static float lastEpaperRh = NAN;
static uint8_t lastEpaperBand = 255;
static bool hasEpaperMeasurement = false;
static uint32_t g_epaperFrameSeq = 0;

static uint8_t epdFrame[EPD_FRAME_BYTES];

static portMUX_TYPE g_epaperMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t g_epaperTaskHandle = nullptr;
static volatile bool g_epaperJobPending = false;
static volatile bool g_epaperRefreshBusy = false;
static uint16_t g_epaperJobCO2 = 0;
static float g_epaperJobTempC = NAN;
static float g_epaperJobRh = NAN;
static bool g_epaperJobForce = false;
static bool g_epaperJobFirstDraw = false;
static bool g_epaperJobPeriodic = false;
static uint8_t g_epaperJobPrevBand = 255;
static uint8_t g_epaperJobBand = 255;
static int16_t g_epaperJobLqi = -1;
static int8_t g_epaperJobRssi = 0;
static char g_epaperJobZbLabel[12] = "NO ZB";

class EpdCanvas : public Adafruit_GFX {
public:
  EpdCanvas() : Adafruit_GFX(EPD_WIDTH, EPD_HEIGHT) {}

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x < 0 || y < 0 || x >= EPD_WIDTH || y >= EPD_HEIGHT) return;
    const uint16_t index = (uint16_t)y * EPD_WIDTH + (uint16_t)x;
    const uint16_t byteIndex = index >> 2;
    const uint8_t shift = (3 - (index & 0x03)) * 2;
    epdFrame[byteIndex] = (epdFrame[byteIndex] & ~(0x03 << shift)) | ((color & 0x03) << shift);
  }

  void clear(uint8_t color) {
    const uint8_t packed = (color & 0x03) * 0x55;
    memset(epdFrame, packed, sizeof(epdFrame));
  }
};

static EpdCanvas epdCanvas;

// ---------- LED ----------
static void setLedRGB(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

static bool updateIdentifyLed() {
  if (!g_identifyUntil) return false;

  const uint32_t now = millis();
  if (now >= g_identifyUntil) {
    g_identifyUntil = 0;
    g_identifyBlinkOn = false;
    return false;
  }

  if (!g_lastIdentifyBlink || now - g_lastIdentifyBlink >= 250) {
    g_lastIdentifyBlink = now;
    g_identifyBlinkOn = !g_identifyBlinkOn;
    pixels.setBrightness(120);
    pixels.setPixelColor(0, g_identifyBlinkOn ? pixels.Color(180, 180, 180) : pixels.Color(0, 0, 0));
    pixels.show();
  }

  return true;
}

static void updateLedIndicator() {
  if (updateIdentifyLed()) return;

  if (!g_ledEnabled) {
    pixels.clear();
    pixels.show();
    return;
  }

  if (!g_zigbeeStarted && !Zigbee.started()) {
    pixels.setBrightness((uint8_t)((uint16_t)g_ledLevel100 * 255 / 100));
    pixels.setPixelColor(0, pixels.Color(140, 0, 0));
    pixels.show();
    return;
  }

  if (!Zigbee.connected() || !g_zclReady || !g_zhaSeen || !g_hasMeasurement) {
    pixels.setBrightness((uint8_t)((uint16_t)g_ledLevel100 * 255 / 100));
    pixels.setPixelColor(0, pixels.Color(0, 0, 120));
    pixels.show();
    return;
  }

  const uint16_t ppm = g_lastCO2;
  const uint8_t maxBr = (uint8_t)((uint16_t)g_ledLevel100 * 255 / 100);

  const float CO2_MIN = 400.0f;
  const float CO2_MAX = (float)red_ppm;

  float x;
  if (ppm <= CO2_MIN) {
    x = 0.0f;
  } else if (ppm >= CO2_MAX) {
    x = 1.0f;
  } else {
    x = (float)(ppm - CO2_MIN) / (CO2_MAX - CO2_MIN);
  }

  const float gamma = 3.0f;
  float k = powf(x, gamma);
  const float k_min = 0.08f;
  k = k_min + (1.0f - k_min) * k;

  uint8_t br = (uint8_t)(maxBr * k);
  pixels.setBrightness(br);

  if (ppm >= red_ppm) {
    bool on = ((millis() / 500) % 2) == 0;
    pixels.setPixelColor(0, on ? pixels.Color(160, 0, 0) : pixels.Color(0, 0, 0));
    pixels.show();
    return;
  }

  uint8_t r = 0, g = 0, b = 0;
  if (ppm < green_ppm) {
    r = 0;   g = 120; b = 0;
  } else if (ppm < yellow_ppm) {
    r = 120; g = 120; b = 0;
  } else if (ppm < orange_ppm) {
    r = 160; g = 60;  b = 0;
  } else {
    r = 160; g = 0;   b = 0;
  }

  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

static void onLedChange(bool state, uint8_t level) {
  const bool oldEnabled = g_ledEnabled;
  const uint8_t oldLevel = g_ledZigbeeLevel;

  g_ledEnabled = state;
  g_ledZigbeeLevel = level;
  g_ledLevel100 = ledZigbeeLevelToPercent(g_ledZigbeeLevel);

  if (oldEnabled != g_ledEnabled || oldLevel != g_ledZigbeeLevel) {
    saveLedState(g_ledEnabled, g_ledZigbeeLevel);
  }

  uint8_t b = (uint8_t)((uint16_t)g_ledLevel100 * 255 / 100);
  pixels.setBrightness(b);

  if (!g_ledEnabled) {
    pixels.clear();
    pixels.show();
  } else {
    updateLedIndicator();
  }
}

static void identify(uint16_t time) {
  Serial.printf("[ZB] identify requested for %u seconds\n", time);
  Serial.flush();

  if (time == 0) {
    g_identifyUntil = 0;
    g_identifyBlinkOn = false;
    updateLedIndicator();
    return;
  }

  g_identifyUntil = millis() + (uint32_t)time * 1000UL;
  g_lastIdentifyBlink = 0;
  g_identifyBlinkOn = false;
  updateIdentifyLed();
}

static void onDisplayRefreshChange(float minutesValue) {
  const float requestedValue = minutesValue;
  if (!isfinite(minutesValue)) {
    minutesValue = DISPLAY_REFRESH_INTERVAL_DEFAULT_MIN;
  }

  int32_t rounded = (int32_t)roundf(minutesValue);
  const uint16_t newMinutes = clampDisplayRefreshMinutes(rounded);
  const bool changed = display_refresh_interval_minutes != newMinutes;

  display_refresh_interval_minutes = newMinutes;
  if (!g_displayRefreshAnalogSyncing) {
    if (changed) {
      saveDisplayRefreshInterval(display_refresh_interval_minutes);
    }
    g_forceEpaperUpdate = g_hasMeasurement;
    if (!isfinite(requestedValue) || fabsf(requestedValue - (float)newMinutes) > 0.01f) {
      g_displayRefreshAnalogSyncPending = true;
    }
  }

  Serial.printf("[CFG] display refresh interval set to %u min, requested=%.2f (%lu ms)\n",
                display_refresh_interval_minutes,
                requestedValue,
                (unsigned long)epaperRefreshIntervalMs());
  Serial.flush();
}

static bool isOurZigbeeEndpoint(uint8_t endpoint) {
  return endpoint == EP_TEMP_HUM ||
         endpoint == EP_CO2 ||
         endpoint == EP_LED_DIM ||
         endpoint == EP_ALARM ||
         endpoint == EP_DISPLAY_REFRESH;
}

static void onZigbeeDefaultResponse(zb_cmd_type_t resp_to_cmd, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
  if (status != ESP_ZB_ZCL_STATUS_SUCCESS || !isOurZigbeeEndpoint(endpoint)) return;

  if (!g_zhaSeen) {
    Serial.printf("[ZB] coordinator responded: endpoint=%u, cluster=0x%04X, resp_to_cmd=0x%02X\n",
                  endpoint, cluster, (uint8_t)resp_to_cmd);
  }
  g_zhaSeen = true;
}

// ---------- E-paper ----------
static uint8_t epdStatusColor(uint16_t ppm) {
  if (ppm >= orange_ppm) return EPD_COLOR_RED;
  if (ppm >= yellow_ppm) return EPD_COLOR_YELLOW;
  return EPD_COLOR_WHITE;
}

static uint8_t airQualityBand(uint16_t ppm) {
  if (ppm < green_ppm) return 0;
  if (ppm < yellow_ppm) return 1;
  if (ppm < orange_ppm) return 2;
  if (ppm < red_ppm) return 3;
  return 4;
}

static uint8_t epdPpmTextColor(uint16_t ppm) {
  if (ppm >= orange_ppm) return EPD_COLOR_RED;
  if (ppm >= green_ppm) return EPD_COLOR_YELLOW;
  return EPD_COLOR_BLACK;
}

static uint8_t epdLinkTextColor(int16_t lqi, const char *label) {
  if (strncmp(label, "NO ZB", 5) == 0) return EPD_COLOR_RED;
  if (lqi < 0) return EPD_COLOR_YELLOW;
  if (lqi < 80) return EPD_COLOR_RED;
  if (lqi < 170) return EPD_COLOR_YELLOW;
  return EPD_COLOR_BLACK;
}

static void drawTemperatureIcon(int16_t x, int16_t y) {
  epdCanvas.fillRoundRect(x + 8, y, 5, 13, 2, EPD_COLOR_YELLOW);
  epdCanvas.fillCircle(x + 10, y + 15, 4, EPD_COLOR_YELLOW);
  epdCanvas.drawRoundRect(x + 7, y, 7, 14, 3, EPD_COLOR_BLACK);
  epdCanvas.drawCircle(x + 10, y + 15, 4, EPD_COLOR_BLACK);
  epdCanvas.drawFastVLine(x + 10, y + 4, 11, EPD_COLOR_BLACK);
  epdCanvas.drawFastHLine(x + 14, y + 4, 3, EPD_COLOR_BLACK);
  epdCanvas.drawFastHLine(x + 14, y + 8, 2, EPD_COLOR_BLACK);
  epdCanvas.drawFastHLine(x + 14, y + 12, 3, EPD_COLOR_BLACK);
}

static void drawHumidityIcon(int16_t x, int16_t y) {
  epdCanvas.fillTriangle(x + 10, y, x + 4, y + 11, x + 16, y + 11, EPD_COLOR_RED);
  epdCanvas.fillCircle(x + 10, y + 12, 6, EPD_COLOR_RED);
  epdCanvas.drawLine(x + 10, y, x + 4, y + 11, EPD_COLOR_BLACK);
  epdCanvas.drawLine(x + 10, y, x + 16, y + 11, EPD_COLOR_BLACK);
  epdCanvas.drawLine(x + 4, y + 11, x + 6, y + 17, EPD_COLOR_BLACK);
  epdCanvas.drawLine(x + 16, y + 11, x + 14, y + 17, EPD_COLOR_BLACK);
  epdCanvas.drawFastHLine(x + 7, y + 18, 7, EPD_COLOR_BLACK);
  epdCanvas.drawPixel(x + 6, y + 17, EPD_COLOR_BLACK);
  epdCanvas.drawPixel(x + 14, y + 17, EPD_COLOR_BLACK);
}

static void printTextAt(int16_t x, int16_t y, uint8_t size, const char *text, uint16_t color, bool bold = true) {
  epdCanvas.setTextSize(size);
  epdCanvas.setTextColor(color);
  epdCanvas.setCursor(x, y);
  epdCanvas.print(text);
  if (bold) {
    epdCanvas.setCursor(x + 1, y);
    epdCanvas.print(text);
  }
}

static void printRightAligned(int16_t right, int16_t y, uint8_t size, const char *text, uint16_t color, bool bold = true) {
  int16_t x1, y1;
  uint16_t w, h;
  epdCanvas.setTextSize(size);
  epdCanvas.setTextColor(color);
  epdCanvas.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  const int16_t x = right - (int16_t)w - (bold ? 1 : 0);
  printTextAt(x, y, size, text, color, bold);
}

static void printEmphasisRightAligned(int16_t right, int16_t y, uint8_t size, const char *text, uint16_t color) {
  int16_t x1, y1;
  uint16_t w, h;
  epdCanvas.setTextSize(size);
  epdCanvas.setTextColor(color);
  epdCanvas.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  const int16_t x = right - (int16_t)w - 1;
  epdCanvas.setCursor(x, y);
  epdCanvas.print(text);
  epdCanvas.setCursor(x + 1, y);
  epdCanvas.print(text);
  epdCanvas.setCursor(x, y + 1);
  epdCanvas.print(text);
}

static uint8_t zigbeeSignalBars(int16_t lqi, int8_t rssi) {
  if (lqi < 0) return 0;

  uint8_t lqiBars = 0;
  if (lqi >= 170) {
    lqiBars = 3;
  } else if (lqi >= 110) {
    lqiBars = 2;
  } else if (lqi >= 70) {
    lqiBars = 1;
  }

  uint8_t rssiBars = lqiBars;
  if (rssi < 0) {
    if (rssi > -65) {
      rssiBars = 3;
    } else if (rssi > -75) {
      rssiBars = 2;
    } else if (rssi > -85) {
      rssiBars = 1;
    } else {
      rssiBars = 0;
    }
  }

  return (lqiBars < rssiBars) ? lqiBars : rssiBars;
}

static void drawSignalBarsIcon(int16_t right, int16_t y, uint8_t bars, uint16_t color) {
  const int16_t iconWidth = 29;
  const int16_t x = right - iconWidth;
  const int16_t baseY = y + 23;

  for (uint8_t i = 0; i < 3; ++i) {
    const int16_t barX = x + (int16_t)i * 10;
    const int16_t barH = 8 + (int16_t)i * 6;
    const int16_t barY = baseY - barH + 1;
    if (bars > i) {
      epdCanvas.fillRect(barX, barY, 7, barH, color);
    }
    epdCanvas.drawRect(barX, barY, 7, barH, EPD_COLOR_BLACK);
  }
}

static void drawRefreshMarker(int16_t x, int16_t y, uint32_t frameSeq, uint8_t color) {
  char marker[8];
  snprintf(marker, sizeof(marker), "R%02lu", (unsigned long)(frameSeq % 100));

  printTextAt(x, y, 2, marker, EPD_COLOR_BLACK);

  const uint8_t dotCount = 6;
  const uint8_t active = frameSeq % dotCount;
  for (uint8_t i = 0; i < dotCount; ++i) {
    const int16_t dotX = x + 4 + (int16_t)i * 10;
    const uint8_t dotColor = (i == active) ? color : EPD_COLOR_BLACK;
    if (i == active) {
      epdCanvas.fillCircle(dotX, y - 7, 3, dotColor);
      epdCanvas.drawCircle(dotX, y - 7, 3, EPD_COLOR_BLACK);
    } else {
      epdCanvas.drawCircle(dotX, y - 7, 2, dotColor);
    }
  }
}

static void drawZigbeeStats(int16_t lqi, int8_t rssi, uint8_t color) {
  drawSignalBarsIcon(EPD_WIDTH - 4, 2, zigbeeSignalBars(lqi, rssi), color);
}

static void formatZigbeeLabel(char *out, size_t len) {
  if (!Zigbee.connected()) {
    snprintf(out, len, "NO ZB");
  } else if (!g_zclReady) {
    snprintf(out, len, "ZB...");
  } else if (g_zigbeeLqi >= 0) {
    snprintf(out, len, "L%d %d", g_zigbeeLqi, g_zigbeeRssi);
  } else {
    snprintf(out, len, "L-- --");
  }
}

static void renderEpaperFrame(uint16_t co2ppm, float tempC, float rh, const char *zigbeeLabel, int16_t lqi, int8_t rssi, uint32_t frameSeq) {
  char buf[24];
  const uint8_t statusColor = epdStatusColor(co2ppm);
  const uint16_t statusText = (statusColor == EPD_COLOR_RED) ? EPD_COLOR_WHITE : EPD_COLOR_BLACK;
  const uint8_t ppmTextColor = epdPpmTextColor(co2ppm);
  const uint8_t linkTextColor = epdLinkTextColor(lqi, zigbeeLabel);

  epdCanvas.clear(EPD_COLOR_WHITE);
  epdCanvas.fillRect(0, 0, EPD_WIDTH, 34, EPD_COLOR_WHITE);
  epdCanvas.drawRect(0, 0, EPD_WIDTH, EPD_HEIGHT, EPD_COLOR_BLACK);
  epdCanvas.drawFastHLine(0, 34, EPD_WIDTH, EPD_COLOR_BLACK);

  epdCanvas.setTextWrap(false);
  drawRefreshMarker(5, 18, frameSeq, ppmTextColor);

  snprintf(buf, sizeof(buf), "%u min", display_refresh_interval_minutes);
  printTextAt(68, 20, 1, buf, EPD_COLOR_BLACK, false);

  drawZigbeeStats(lqi, rssi, linkTextColor);

  snprintf(buf, sizeof(buf), "%u", co2ppm);
  printTextAt(10, 46, (co2ppm > 9999) ? 3 : 4, buf, ppmTextColor);

  printTextAt(110, 66, 2, "ppm", ppmTextColor);

  epdCanvas.drawFastHLine(8, 90, EPD_WIDTH - 16, EPD_COLOR_BLACK);

  snprintf(buf, sizeof(buf), "%.1f C", tempC);
  drawTemperatureIcon(8, 94);
  printTextAt(34, 99, 2, buf, EPD_COLOR_BLACK);

  snprintf(buf, sizeof(buf), "%.0f %%", rh);
  drawHumidityIcon(8, 116);
  printTextAt(34, 118, 2, buf, EPD_COLOR_BLACK);

  epdCanvas.fillRect(0, 136, EPD_WIDTH, 16, statusColor);
  epdCanvas.drawFastHLine(0, 136, EPD_WIDTH, EPD_COLOR_BLACK);
  const char *statusLabel = "AIR GOOD";
  if (co2ppm < green_ppm) {
    statusLabel = "AIR GOOD";
  } else if (co2ppm < yellow_ppm) {
    statusLabel = "AIR OK";
  } else if (co2ppm < orange_ppm) {
    statusLabel = "VENTILATE";
  } else if (co2ppm < red_ppm) {
    statusLabel = "HIGH CO2";
  } else {
    statusLabel = "ALARM";
  }
  printTextAt(8, 141, 1, statusLabel, statusText);
}

static bool epaperBusyOrPending() {
  portENTER_CRITICAL(&g_epaperMux);
  const bool busy = g_epaperRefreshBusy || g_epaperJobPending;
  portEXIT_CRITICAL(&g_epaperMux);
  return busy;
}

static bool queueEpaperRefresh(uint16_t co2ppm, float tempC, float rh, bool force, bool firstDraw, bool periodicReady,
                               uint8_t previousBand, uint8_t currentBand, const char *zigbeeLabel) {
  portENTER_CRITICAL(&g_epaperMux);
  if (g_epaperRefreshBusy || g_epaperJobPending) {
    portEXIT_CRITICAL(&g_epaperMux);
    return false;
  }

  g_epaperJobCO2 = co2ppm;
  g_epaperJobTempC = tempC;
  g_epaperJobRh = rh;
  g_epaperJobForce = force;
  g_epaperJobFirstDraw = firstDraw;
  g_epaperJobPeriodic = periodicReady;
  g_epaperJobPrevBand = previousBand;
  g_epaperJobBand = currentBand;
  g_epaperJobLqi = g_zigbeeLqi;
  g_epaperJobRssi = g_zigbeeRssi;
  snprintf(g_epaperJobZbLabel, sizeof(g_epaperJobZbLabel), "%s", zigbeeLabel);
  g_epaperJobPending = true;
  portEXIT_CRITICAL(&g_epaperMux);

  if (g_epaperTaskHandle) {
    xTaskNotifyGive(g_epaperTaskHandle);
  }
  return true;
}

static bool takeEpaperRefreshJob(uint16_t &co2ppm, float &tempC, float &rh, bool &force, bool &firstDraw, bool &periodicReady,
                                 uint8_t &previousBand, uint8_t &currentBand, int16_t &lqi, int8_t &rssi,
                                 char *zigbeeLabel, size_t zigbeeLabelLen) {
  portENTER_CRITICAL(&g_epaperMux);
  if (!g_epaperJobPending) {
    portEXIT_CRITICAL(&g_epaperMux);
    return false;
  }

  co2ppm = g_epaperJobCO2;
  tempC = g_epaperJobTempC;
  rh = g_epaperJobRh;
  force = g_epaperJobForce;
  firstDraw = g_epaperJobFirstDraw;
  periodicReady = g_epaperJobPeriodic;
  previousBand = g_epaperJobPrevBand;
  currentBand = g_epaperJobBand;
  lqi = g_epaperJobLqi;
  rssi = g_epaperJobRssi;
  snprintf(zigbeeLabel, zigbeeLabelLen, "%s", g_epaperJobZbLabel);
  g_epaperJobPending = false;
  g_epaperRefreshBusy = true;
  portEXIT_CRITICAL(&g_epaperMux);
  return true;
}

static void finishEpaperRefreshJob() {
  portENTER_CRITICAL(&g_epaperMux);
  g_epaperRefreshBusy = false;
  portEXIT_CRITICAL(&g_epaperMux);
}

static void runEpaperRefresh(uint16_t co2ppm, float tempC, float rh, bool force, bool firstDraw, bool periodicReady,
                             uint8_t previousBand, uint8_t currentBand, int16_t lqi, int8_t rssi, const char *zigbeeLabel) {
  const uint32_t refreshStart = millis();
  Serial.printf("[EPD] cycle start: co2=%u ppm, temp=%.2f C, rh=%.2f %%, force=%u, firstDraw=%u, periodicReady=%u, band=%u->%u, lqi=%d, rssi=%d, label=%s\n",
                co2ppm, tempC, rh, force ? 1 : 0, firstDraw ? 1 : 0, periodicReady ? 1 : 0, previousBand, currentBand, lqi, rssi, zigbeeLabel);
  Serial.println("[EPD] note: full 4-color refresh is slow, but it runs in its own task so sensor/Zigbee loop keeps working.");
  Serial.flush();

  const uint32_t renderStart = millis();
  const uint32_t frameSeq = ++g_epaperFrameSeq;
  renderEpaperFrame(co2ppm, tempC, rh, zigbeeLabel, lqi, rssi, frameSeq);
  Serial.printf("[EPD] render frame done, frame=%lu, elapsed=%lu ms\n",
                (unsigned long)frameSeq,
                (unsigned long)(millis() - renderStart));
  Serial.flush();

  EPD_init();
  EPD_displayNative(epdFrame);
  EPD_sleep();

  lastEpaper = millis();
  lastEpaperCO2 = co2ppm;
  lastEpaperTempC = tempC;
  lastEpaperRh = rh;
  lastEpaperBand = currentBand;
  hasEpaperMeasurement = true;

  Serial.printf("[EPD] cycle done, total=%lu ms\n", (unsigned long)(millis() - refreshStart));
  Serial.flush();
}

static void epaperTask(void *parameter) {
  (void)parameter;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));

    uint16_t co2ppm = 0;
    float tempC = NAN;
    float rh = NAN;
    bool force = false;
    bool firstDraw = false;
    bool periodicReady = false;
    uint8_t previousBand = 255;
    uint8_t currentBand = 255;
    int16_t lqi = -1;
    int8_t rssi = 0;
    char zigbeeLabel[12] = "NO ZB";

    if (!takeEpaperRefreshJob(co2ppm, tempC, rh, force, firstDraw, periodicReady, previousBand, currentBand, lqi, rssi,
                              zigbeeLabel, sizeof(zigbeeLabel))) {
      continue;
    }

    runEpaperRefresh(co2ppm, tempC, rh, force, firstDraw, periodicReady, previousBand, currentBand, lqi, rssi, zigbeeLabel);
    finishEpaperRefreshJob();
  }
}

static void updateEpaper(uint16_t co2ppm, float tempC, float rh, bool force = false) {
  const uint32_t now = millis();
  const uint32_t age = lastEpaper ? now - lastEpaper : 0;
  const uint32_t refreshIntervalMs = epaperRefreshIntervalMs();
  const uint8_t currentBand = airQualityBand(co2ppm);
  const bool firstDraw = !hasEpaperMeasurement;
  const bool changeReady = !lastEpaper || (age >= refreshIntervalMs);
  const bool periodicReady = lastEpaper && (age >= refreshIntervalMs);
  const bool bandChanged = hasEpaperMeasurement && currentBand != lastEpaperBand;
  const bool valueChanged = !hasEpaperMeasurement
                            || abs((int32_t)co2ppm - (int32_t)lastEpaperCO2) >= EPAPER_CO2_DELTA_PPM
                            || fabsf(tempC - lastEpaperTempC) >= EPAPER_TEMP_DELTA_C
                            || fabsf(rh - lastEpaperRh) >= EPAPER_RH_DELTA;

  const bool shouldRefresh = force || firstDraw || periodicReady || ((bandChanged || valueChanged) && changeReady);

  if (!shouldRefresh) {
    if (!lastEpaperSkipLog || now - lastEpaperSkipLog >= EPAPER_SKIP_LOG_MS) {
      lastEpaperSkipLog = now;
      Serial.printf("[EPD] skip refresh: auto_policy=change/manual/periodic, force=%u, firstDraw=%u, changeReady=%u, periodicReady=%u, busy=%u, age=%lu ms, refresh_interval=%lu ms, bandChanged=%u, valueChanged=%u, band=%u->%u, dCO2=%ld, dT=%.2f, dRH=%.2f\n",
                    force ? 1 : 0,
                    firstDraw ? 1 : 0,
                    changeReady ? 1 : 0,
                    periodicReady ? 1 : 0,
                    epaperBusyOrPending() ? 1 : 0,
                    (unsigned long)age,
                    (unsigned long)refreshIntervalMs,
                    bandChanged ? 1 : 0,
                    valueChanged ? 1 : 0,
                    lastEpaperBand,
                    currentBand,
                    hasEpaperMeasurement ? (long)((int32_t)co2ppm - (int32_t)lastEpaperCO2) : 0,
                    hasEpaperMeasurement ? (tempC - lastEpaperTempC) : 0.0f,
                    hasEpaperMeasurement ? (rh - lastEpaperRh) : 0.0f);
      Serial.flush();
    }
    return;
  }

  char zbLabel[12];
  formatZigbeeLabel(zbLabel, sizeof(zbLabel));

  if (!g_epaperTaskHandle) {
    Serial.println("[EPD] task is not running; falling back to blocking refresh.");
    Serial.flush();
    runEpaperRefresh(co2ppm, tempC, rh, force, firstDraw, periodicReady, lastEpaperBand, currentBand, g_zigbeeLqi, g_zigbeeRssi, zbLabel);
    return;
  }

  if (!queueEpaperRefresh(co2ppm, tempC, rh, force, firstDraw, periodicReady, lastEpaperBand, currentBand, zbLabel)) {
    if (!lastEpaperSkipLog || now - lastEpaperSkipLog >= EPAPER_SKIP_LOG_MS) {
      lastEpaperSkipLog = now;
      Serial.printf("[EPD] refresh request skipped: previous full refresh still running, age=%lu ms, co2=%u, label=%s\n",
                    (unsigned long)age, co2ppm, zbLabel);
      Serial.flush();
    }
    return;
  }

  Serial.printf("[EPD] refresh queued: co2=%u ppm, temp=%.2f C, rh=%.2f %%, force=%u, firstDraw=%u, changeReady=%u, periodicReady=%u, refresh_interval=%lu ms, bandChanged=%u, valueChanged=%u, label=%s\n",
                co2ppm, tempC, rh, force ? 1 : 0, firstDraw ? 1 : 0, changeReady ? 1 : 0, periodicReady ? 1 : 0,
                (unsigned long)refreshIntervalMs, bandChanged ? 1 : 0, valueChanged ? 1 : 0, zbLabel);
  Serial.flush();
}

// ---------- Zigbee ----------
static bool zigbeeUsable() {
  return g_zigbeeStarted && g_zclReady && Zigbee.connected();
}

static void applyZigbeeTxPower(const char *reason) {
  esp_err_t err = esp_ieee802154_set_txpower(ZIGBEE_TX_POWER_DBM);
  int8_t actualPower = esp_ieee802154_get_txpower();

  if (err == ESP_OK) {
    Serial.printf("[ZB] TX power requested: %d dBm (%s), current: %d dBm\n",
                  ZIGBEE_TX_POWER_DBM, reason, actualPower);
  } else {
    Serial.printf("[ZB] TX power request failed: %s (%s), current: %d dBm\n",
                  esp_err_to_name(err), reason, actualPower);
  }
  Serial.flush();
}

static void configureZigbeeTxPower() {
  if (g_zigbeeTxPowerConfigured) return;
  if (!g_zigbeeStarted && !Zigbee.started()) return;

  applyZigbeeTxPower("stack started");
  g_zigbeeTxPowerConfigured = true;
}

static void resetZigbeeLinkStats() {
  g_zigbeeLqi = -1;
  g_zigbeeRssi = 0;
  g_zigbeeChannel = 0;
  g_zigbeeShortAddr = 0xFFFF;
  g_zigbeeParentShortAddr = 0xFFFF;
  g_lastZigbeeLqiPoll = 0;
}

static void updateZigbeeLinkStats(uint32_t now, bool force = false) {
  if (!Zigbee.connected()) {
    if (g_zigbeeLqi >= 0 || g_zigbeeShortAddr != 0xFFFF) {
      Serial.println("[ZB] link stats reset: disconnected");
      Serial.flush();
    }
    resetZigbeeLinkStats();
    return;
  }

  if (!force && g_lastZigbeeLqiPoll && now - g_lastZigbeeLqiPoll < ZIGBEE_LQI_POLL_MS) return;
  g_lastZigbeeLqiPoll = now;

  if (!esp_zb_lock_acquire(pdMS_TO_TICKS(100))) {
    Serial.println("[ZB] link stats skipped: Zigbee lock timeout");
    Serial.flush();
    return;
  }

  g_zigbeeChannel = esp_zb_get_current_channel();
  g_zigbeeShortAddr = esp_zb_get_short_address();

  esp_zb_nwk_info_iterator_t iterator = ESP_ZB_NWK_INFO_ITERATOR_INIT;
  esp_zb_nwk_neighbor_info_t neighbor;
  esp_err_t err = ESP_OK;
  uint8_t count = 0;
  bool found = false;
  bool foundParent = false;
  uint8_t selectedLqi = 0;
  int8_t selectedRssi = 0;
  uint16_t selectedShort = 0xFFFF;
  uint8_t selectedRelationship = 0xFF;

  while ((err = esp_zb_nwk_get_next_neighbor(&iterator, &neighbor)) == ESP_OK) {
    ++count;
    Serial.printf("[ZB] neighbor: short=0x%04X, relationship=%u, lqi=%u, rssi=%d, age=%u, outgoing_cost=%u\n",
                  neighbor.short_addr,
                  neighbor.relationship,
                  neighbor.lqi,
                  neighbor.rssi,
                  neighbor.age,
                  neighbor.outgoing_cost);

    const bool isParent = neighbor.relationship == ESP_ZB_NWK_RELATIONSHIP_PARENT;
    if (isParent || (!found && !foundParent) || (!foundParent && neighbor.lqi > selectedLqi)) {
      found = true;
      foundParent = isParent;
      selectedLqi = neighbor.lqi;
      selectedRssi = neighbor.rssi;
      selectedShort = neighbor.short_addr;
      selectedRelationship = neighbor.relationship;
      if (isParent) break;
    }
  }

  esp_zb_lock_release();

  if (found) {
    g_zigbeeLqi = selectedLqi;
    g_zigbeeRssi = selectedRssi;
    g_zigbeeParentShortAddr = selectedShort;
    Serial.printf("[ZB] link: short=0x%04X, channel=%u, %s=0x%04X, lqi=%d, rssi=%d dBm, relationship=%u, neighbors=%u\n",
                  g_zigbeeShortAddr,
                  g_zigbeeChannel,
                  foundParent ? "parent" : "best_neighbor",
                  g_zigbeeParentShortAddr,
                  g_zigbeeLqi,
                  g_zigbeeRssi,
                  selectedRelationship,
                  count);
  } else {
    g_zigbeeLqi = -1;
    g_zigbeeRssi = 0;
    g_zigbeeParentShortAddr = 0xFFFF;
    if (err == ESP_ERR_NOT_FOUND) {
      Serial.printf("[ZB] link: no neighbors yet, short=0x%04X, channel=%u\n", g_zigbeeShortAddr, g_zigbeeChannel);
    } else {
      Serial.printf("[ZB] link: neighbor scan failed, err=0x%X, short=0x%04X, channel=%u\n",
                    (unsigned)err,
                    g_zigbeeShortAddr,
                    g_zigbeeChannel);
    }
  }
  Serial.flush();
}

static void configureZigbeeReporting() {
  if (g_zigbeeReportingConfigured) return;

  zbTempHum.setReporting(10, 300, 0.2f);
  zbTempHum.setHumidityReporting(10, 300, 1.0f);
  zbCO2.setReporting(0, 30, 0);
  g_zigbeeReportingConfigured = true;
  Serial.println("Zigbee reporting configured.");
}

static void updateCo2Alarm(uint16_t ppm) {
  if (!zigbeeUsable()) return;

  bool newAlarm = (ppm >= red_ppm);
  if (newAlarm == g_alarm) return;

  g_alarm = newAlarm;
  zbAlarm.setBinaryInput(g_alarm);
}

static void updateZigbeeReady() {
  const bool startedNow = Zigbee.started();
  const bool connectedNow = Zigbee.connected();

  if (startedNow && !g_zigbeeStarted) {
    g_zigbeeStarted = true;
    Serial.println("Zigbee stack became started after begin timeout.");
    configureZigbeeTxPower();
    configureZigbeeReporting();
  }

  configureZigbeeTxPower();

  if (connectedNow != g_lastZigbeeConnected) {
    g_lastZigbeeConnected = connectedNow;
    g_forceEpaperUpdate = g_hasMeasurement;
    Serial.printf("Zigbee connection state changed: %s\n", connectedNow ? "connected" : "disconnected");
    g_zhaSeen = false;
    if (connectedNow) {
      applyZigbeeTxPower("joined coordinator");
      g_zigbeeTxPowerAppliedAfterJoin = true;
    } else {
      g_zigbeeTxPowerAppliedAfterJoin = false;
    }
    if (!connectedNow) {
      g_zclReady = false;
      g_connectedAt = 0;
      resetZigbeeLinkStats();
    }
  }

  if (connectedNow && !g_zigbeeTxPowerAppliedAfterJoin) {
    applyZigbeeTxPower("joined coordinator");
    g_zigbeeTxPowerAppliedAfterJoin = true;
  }

  if (!g_zigbeeStarted || g_zclReady) return;

  if (!connectedNow) {
    g_connectedAt = 0;
    return;
  }

  if (!g_connectedAt) {
    g_connectedAt = millis();
    Serial.println("Zigbee connected, waiting for ZCL...");
    return;
  }

  if (millis() - g_connectedAt < ZIGBEE_READY_DELAY_MS) return;

  g_zclReady = true;
  g_forceEpaperUpdate = g_hasMeasurement;
  lastReport = millis() - ZB_REPORT_MS;
  updateZigbeeLinkStats(millis(), true);
  zbAlarm.setBinaryInput(false);
  zbLedDim.setLight(g_ledEnabled, g_ledZigbeeLevel);
  zbLedDim.restoreLight();
  syncDisplayRefreshAnalogOutput("zcl-ready", true);

  Serial.println("Zigbee ZCL ready.");
}

static void publishZigbeeValues(uint16_t co2ppm, float tempC, float rh) {
  if (!zigbeeUsable()) return;

  zbCO2.setCarbonDioxide((float)co2ppm);
  zbTempHum.setTemperature(tempC);
  zbTempHum.setHumidity(rh);
}

static void reportZigbeeValues(uint32_t now) {
  if (!zigbeeUsable()) return;
  if (now - lastReport < ZB_REPORT_MS) return;

  lastReport = now;
  zbCO2.report();
  zbTempHum.report();
  zbDisplayRefresh.reportAnalogOutput();
  Serial.println("Zigbee report sent.");
}

static void syncDisplayRefreshAnalogOutput(const char *phase, bool report) {
  g_displayRefreshAnalogSyncing = true;
  const bool setOk = zbDisplayRefresh.setAnalogOutput((float)display_refresh_interval_minutes);
  g_displayRefreshAnalogSyncing = false;

  bool reportOk = true;
  if (report) {
    reportOk = zbDisplayRefresh.reportAnalogOutput();
  }

  Serial.printf("[ZB] display refresh AnalogOutput sync: phase=%s, value=%u min, set=%s, report=%s\n",
                phase,
                display_refresh_interval_minutes,
                setOk ? "ok" : "FAIL",
                report ? (reportOk ? "ok" : "FAIL") : "skip");
  Serial.flush();
}

static void serviceDisplayRefreshAnalogSync() {
  if (!g_displayRefreshAnalogSyncPending || !zigbeeUsable()) return;
  g_displayRefreshAnalogSyncPending = false;
  syncDisplayRefreshAnalogOutput("rounded-int", true);
}

static void addZigbeeEndpointChecked(const char *label, ZigbeeEP *endpoint) {
  const bool ok = Zigbee.addEndpoint(endpoint);
  Serial.printf("[ZB] add endpoint: %s, ep=%u, result=%s\n",
                label,
                endpoint->getEndpoint(),
                ok ? "ok" : "FAIL");
  Serial.flush();
}

// ---------- SCD4x ----------
static bool initScd4x() {
  Wire.begin(I2C_SDA, I2C_SCL);
  scd4x.begin(Wire, SCD4X_I2C_ADDR);

  uint16_t err = 0;
  char errMsg[64];

  (void)scd4x.stopPeriodicMeasurement();
  delay(50);

  err = scd4x.setAutomaticSelfCalibrationTarget(400);
  if (err) {
    errorToString(err, errMsg, sizeof(errMsg));
    Serial.printf("SCD4x setASC target failed: %s\n", errMsg);
  }

  err = scd4x.startLowPowerPeriodicMeasurement();
  if (err) {
    errorToString(err, errMsg, sizeof(errMsg));
    Serial.printf("SCD4x startLowPowerPeriodicMeasurement failed: %s\n", errMsg);
    return false;
  }

  return true;
}

static bool readScd4x(uint16_t &co2ppm, float &tempC, float &rh) {
  int16_t err = 0;
  char errMsg[64];

  bool dataReady = false;
  err = scd4x.getDataReadyStatus(dataReady);
  if (err) {
    errorToString(err, errMsg, sizeof(errMsg));
    Serial.printf("SCD4x getDataReadyStatus failed: %s\n", errMsg);
    return false;
  }
  if (!dataReady) return false;

  err = scd4x.readMeasurement(co2ppm, tempC, rh);
  if (err) {
    errorToString(err, errMsg, sizeof(errMsg));
    Serial.printf("SCD4x readMeasurement failed: %s\n", errMsg);
    return false;
  }

  if (co2ppm == 0) return false;
  return true;
}

// ---------- Button ----------
static void handleButton() {
  if (digitalRead(button) != LOW) return;

  delay(100);
  uint32_t start = millis();

  while (digitalRead(button) == LOW) {
    delay(50);
    if (millis() - start > 3000) {
      Serial.println("Factory reset Zigbee NVRAM requested. Release BOOT to reboot...");
      setLedRGB(120, 0, 0);
      Serial.flush();
      Zigbee.factoryReset(false);
      delay(1000);
      while (digitalRead(button) == LOW) {
        delay(20);
      }
      delay(100);
      Serial.println("Rebooting after Zigbee factory reset.");
      Serial.flush();
      ESP.restart();
    }
  }

  if (g_hasMeasurement) {
    Serial.println("Manual e-paper refresh requested by button.");
    updateEpaper(g_lastCO2, g_lastTempC, g_lastRh, true);
  } else {
    Serial.println("Manual e-paper refresh skipped: no measurement yet.");
  }

  if (zigbeeUsable()) {
    Serial.println("Manual Zigbee report()");
    zbTempHum.report();
    zbCO2.report();
  } else {
    Serial.println("Manual Zigbee report skipped: Zigbee is not ready.");
  }
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  const uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 2000) {
    delay(10);
  }
  delay(200);

  Serial.println();
  Serial.println("Boot: ESP32-H2 SCD4x Zigbee + GDEY0154F51");
  loadDisplayRefreshInterval();
  loadLedState(g_ledEnabled, g_ledZigbeeLevel);
  g_ledLevel100 = ledZigbeeLevelToPercent(g_ledZigbeeLevel);
  Serial.printf("Pins: I2C SDA=%u SCL=%u, LED=%u, BOOT=%u, EPD BUSY=%u RST=%u DC=%u CS=%u SCK=%u MOSI=%u\n",
                I2C_SDA, I2C_SCL, WS2812_GPIO, button, EPD_BUSY_PIN, EPD_RST_PIN, EPD_DC_PIN, EPD_CS_PIN, EPD_SCK_PIN, EPD_MOSI_PIN);
  Serial.printf("Intervals: sensor_poll=%lu ms, zigbee_report=%lu ms, display_refresh=%u min (%lu ms), lqi_poll=%lu ms\n",
                (unsigned long)SENSOR_POLL_MS,
                (unsigned long)ZB_REPORT_MS,
                display_refresh_interval_minutes,
                (unsigned long)epaperRefreshIntervalMs(),
                (unsigned long)ZIGBEE_LQI_POLL_MS);
  Serial.printf("EPD deltas: CO2=%u ppm, T=%.1f C, RH=%.1f %%\n",
                EPAPER_CO2_DELTA_PPM, EPAPER_TEMP_DELTA_C, EPAPER_RH_DELTA);
  Serial.println("Serial check: if you see this on COM5, USB CDC logging is active.");
  Serial.flush();

  pinMode(button, INPUT_PULLUP);

  pixels.begin();
  pixels.setBrightness((uint8_t)((uint16_t)g_ledLevel100 * 255 / 100));
  pixels.clear();
  pixels.show();
  updateLedIndicator();

  EPD_ioInit();
  if (xTaskCreate(epaperTask, "epaper", 6144, nullptr, 1, &g_epaperTaskHandle) == pdPASS) {
    Serial.println("[EPD] background task started.");
  } else {
    g_epaperTaskHandle = nullptr;
    Serial.println("[EPD] background task failed; refresh will block the main loop.");
  }
  Serial.flush();

  if (!initScd4x()) {
    setLedRGB(80, 0, 80);
  }

  zbTempHum.setManufacturerAndModel("SAVA_Lab", "ESP32H2_SCD4x");
  zbTempHum.setMinMaxValue(-10, 60);
  zbTempHum.setTolerance(0.2f);
  zbTempHum.addHumiditySensor(0, 100, 1.0f);

  Zigbee.onGlobalDefaultResponse(onZigbeeDefaultResponse);

  zbCO2.setManufacturerAndModel("SAVA_Lab", "ESP32H2_SCD4x");
  zbCO2.setMinMaxValue(0, 10000);
  zbCO2.setTolerance(50);

  zbLedDim.setManufacturerAndModel("SAVA_Lab", "ESP32H2_SCD4x_LED");
  zbLedDim.onLightChange(onLedChange);
  zbLedDim.onIdentify(identify);

  zbDisplayRefresh.setManufacturerAndModel("SAVA_Lab", "ESP32H2_SCD4x_Display");
  zbDisplayRefresh.addAnalogOutput();
  zbDisplayRefresh.setAnalogOutputApplication(ESP_ZB_ZCL_AO_TIME_OTHER);
  zbDisplayRefresh.setAnalogOutputDescription("Refresh min");
  zbDisplayRefresh.setAnalogOutputResolution(1.0f);
  zbDisplayRefresh.setAnalogOutputMinMax((float)DISPLAY_REFRESH_INTERVAL_MIN, (float)DISPLAY_REFRESH_INTERVAL_MAX);
  zbDisplayRefresh.onAnalogOutputChange(onDisplayRefreshChange);

  zbAlarm.addBinaryInput();
  zbAlarm.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_SECURITY_CARBON_DIOXIDE_DETECTION);
  zbAlarm.setBinaryInputDescription("CO2 alarm");

  zbTempHum.onIdentify(identify);
  zbCO2.onIdentify(identify);
  zbAlarm.onIdentify(identify);
  zbDisplayRefresh.onIdentify(identify);

  addZigbeeEndpointChecked("CO2 alarm BinaryInput", &zbAlarm);
  addZigbeeEndpointChecked("LED DimmableLight", &zbLedDim);
  addZigbeeEndpointChecked("Display refresh AnalogOutput", &zbDisplayRefresh);
  addZigbeeEndpointChecked("Temperature/Humidity", &zbTempHum);
  addZigbeeEndpointChecked("CO2 measurement", &zbCO2);

  Serial.println("Starting Zigbee...");
  Zigbee.setTimeout(5000);
  g_zigbeeStarted = Zigbee.begin();
  if (g_zigbeeStarted) {
    Serial.println("Zigbee stack started. Measurements continue while pairing/connecting.");
    configureZigbeeTxPower();
    syncDisplayRefreshAnalogOutput("stack-started", false);
    configureZigbeeReporting();
  } else {
    Serial.println("Zigbee start failed/timeout. Measurements continue locally.");
  }

  updateLedIndicator();
}

void loop() {
  handleButton();
  updateZigbeeReady();
  serviceDisplayRefreshAnalogSync();
  updateLedIndicator();

  const uint32_t now = millis();
  updateZigbeeLinkStats(now);

  if (now - lastPoll >= SENSOR_POLL_MS) {
    lastPoll = now;

    uint16_t co2ppm = 0;
    float tempC = NAN;
    float rh = NAN;

    if (readScd4x(co2ppm, tempC, rh)) {
      Serial.printf("SCD4x: CO2=%u ppm, T=%.2f C, RH=%.2f %%\n", co2ppm, tempC, rh);

      g_lastCO2 = co2ppm;
      g_lastTempC = tempC;
      g_lastRh = rh;
      g_hasMeasurement = true;

      publishZigbeeValues(co2ppm, tempC, rh);
      updateCo2Alarm(co2ppm);
      reportZigbeeValues(now);
      updateEpaper(co2ppm, tempC, rh, g_forceEpaperUpdate);
      g_forceEpaperUpdate = false;
    }
  }

  delay(20);
}
