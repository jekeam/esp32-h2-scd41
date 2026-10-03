#include "../EpaperRefreshPolicy.h"

using namespace epaper_policy;

constexpr bool sixSleepCycles() {
  State state = defaultState();
  recordSuccess(state, 650, 24.0f, 45.0f, 0);
  for (int i = 1; i <= 6; ++i) {
    beginWake(state, true, true);
    if (shouldRefresh(state, 0, 0) != (i == 6)) return false;
    if (state.lastDisplayedCo2 != 650) return false;
  }
  recordSuccess(state, 700, 25.0f, 46.0f, 0);
  return state.cyclesSinceEpd == 0 && !shouldRefresh(state, 0, 0) &&
         state.lastDisplayedCo2 == 700;
}

constexpr bool coldBootAndCorruption() {
  State state = defaultState();
  recordSuccess(state, 650, 24.0f, 45.0f, 0);
  beginWake(state, false, false);
  if (!shouldRefresh(state, 0, 0)) return false;
  recordSuccess(state, 650, 24.0f, 45.0f, 0);
  state.magic = 0;
  beginWake(state, true, true);
  if (state.hasFrame || state.cyclesSinceEpd != 0) return false;
  recordSuccess(state, 650, 24.0f, 45.0f, 0);
  state.version = 99;
  beginWake(state, true, true);
  return !state.hasFrame && valid(state);
}

constexpr bool buttonWakeAndOffline() {
  State state = defaultState();
  recordSuccess(state, 650, 24.0f, 45.0f, 0);
  for (int i = 0; i < 10; ++i) beginWake(state, true, false);
  if (state.cyclesSinceEpd != 0 || shouldRefresh(state, 0, 0)) return false;
  for (int i = 0; i < 5; ++i) beginWake(state, true, true);
  return !shouldRefresh(state, 0, kCycleMs - 1) && shouldRefresh(state, 0, kCycleMs);
}

constexpr bool zoneAndFailure() {
  State state = defaultState();
  recordSuccess(state, 650, 24.0f, 45.0f, 0);
  if (!shouldRefresh(state, 1, 0) || shouldRefresh(state, 0, 0)) return false;
  state.retryNeeded = true;
  beginWake(state, true, true);
  if (!shouldRefresh(state, 0, 0) || state.lastDisplayedCo2 != 650) return false;
  for (int i = 0; i < 300; ++i) beginWake(state, true, true);
  if (state.cyclesSinceEpd != 6 || !valid(state)) return false;
  recordSuccess(state, 1300, 26.0f, 47.0f, 2);
  return !state.retryNeeded && !shouldRefresh(state, 2, 0);
}

static_assert(sixSleepCycles(), "Refresh on sixth timer wake, not seventh");
static_assert(coldBootAndCorruption(), "Cold boot or invalid RTC must redraw");
static_assert(buttonWakeAndOffline(), "BOOT is not a full sleep cycle; offline time still counts");
static_assert(zoneAndFailure(), "Only success replaces the displayed frame and clears retries");
