#pragma once

#include <stdint.h>

namespace epaper_policy {
constexpr uint32_t kMagic = 0x45504431;
constexpr uint16_t kVersion = 1;
constexpr uint32_t kCycleMs = 10UL * 60UL * 1000UL;
constexpr uint8_t kRefreshCycles = 6;

struct State {
  uint32_t magic;
  uint16_t version;
  uint8_t cyclesSinceEpd;
  uint8_t lastDisplayedZone;
  uint16_t lastDisplayedCo2;
  float lastDisplayedTemp;
  float lastDisplayedRh;
  bool hasFrame;
  bool retryNeeded;
};

constexpr State defaultState() {
  return {kMagic, kVersion, 0, 0, 0, 0.0f, 0.0f, false, false};
}

constexpr bool valid(const State &state) {
  return state.magic == kMagic && state.version == kVersion &&
         state.cyclesSinceEpd <= kRefreshCycles &&
         (!state.hasFrame || (state.lastDisplayedZone <= 4 && state.lastDisplayedCo2 != 0));
}

constexpr void beginWake(State &state, bool deepSleepWake, bool timerWake) {
  if (!deepSleepWake || !valid(state)) {
    state = defaultState();
  } else if (timerWake && state.cyclesSinceEpd < kRefreshCycles) {
    ++state.cyclesSinceEpd;
  }
}

constexpr bool periodicDue(const State &state, uint32_t awakeSinceFrameMs) {
  return state.cyclesSinceEpd >= kRefreshCycles ||
         awakeSinceFrameMs >= (kRefreshCycles - state.cyclesSinceEpd) * kCycleMs;
}

constexpr bool shouldRefresh(const State &state, uint8_t zone, uint32_t awakeSinceFrameMs) {
  return !state.hasFrame || state.retryNeeded || zone != state.lastDisplayedZone ||
         periodicDue(state, awakeSinceFrameMs);
}

constexpr void recordSuccess(State &state, uint16_t co2, float temp, float rh, uint8_t zone) {
  state.lastDisplayedCo2 = co2;
  state.lastDisplayedTemp = temp;
  state.lastDisplayedRh = rh;
  state.lastDisplayedZone = zone;
  state.cyclesSinceEpd = 0;
  state.hasFrame = true;
  state.retryNeeded = false;
}
}  // namespace epaper_policy
